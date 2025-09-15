/*
Stromvelo-Arduino-Dynamixel.ino

Dieses Programm für einen Arduino Mega mit angeschlossenem Dynamixel AX-12 Servo
steuert eine visuelle Anzeigescheibe basierend auf der gemessenen Stromproduktion
eines Fahrrad-Generators. Der erzeugte Strom wird mit einem ACS712 Stromsensor
gemessen und die Scheibe entsprechend positioniert, um den Nutzern visuelles
Feedback über ihre Energieproduktion zu geben.

Hardware-Setup:
- Arduino Mega 2560
- Dynamixel AX-12 Servo (ID 1) für Hauptanzeige
- ACS712 Stromsensor (20A Modul) an Pin A0
- Kommunikation über Pin 50 (unterstützt Pin Change Interrupts)

Funktionsweise:
1. Auto-Kalibrierung beim Start (5 Sekunden Leerlauf-Messung)
2. Kontinuierliche Strommessung alle 500ms
3. Glättung der Messwerte zur Rauschunterdrückung  
4. Akkumulation der Stromwerte während aktiver Produktion
5. Servo-Position basierend auf gesammelter Energie
6. Automatisches Zurücksetzen nach Inaktivität

Zur Ansteuerung der Servos wird keine Dynamixel-Shield verwendet.
Die direkte Kommunikation basiert auf Code von Akira's DynamixelMxMonitorBlock.ino

 *****************************************************************************
Original DynamixelMxMonitorBlock.ino
written by Akira

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

 *****************************************************************************
 Description:
 This library implements all required operations to drive Dynamixel servos.
 Visit http://support.robotis.com/en/product/dynamixel/dxl_communication.htm 
 for Dynamixel communication protocol details.

 Hardware Notes:
  - Uses softHalfDuplexSerial library with Pin Change Interrupt support
  - Pin 50 used for Dynamixel communication (single wire, half-duplex)
  - Proper grounding between Arduino and Dynamixel power supply required
  - Servo baudrate: 57600 bps
  - Default servo ID: 1
  - Blocking implementation: waits for servo responses
*/

#include "src\SoftHalfDuplexSerial.h"
#include "src\DynamixelAx.h"

// Kommunikation Setup
softHalfDuplexSerial port(50); // Pin 50 für Mega (unterstützt Pin Change Interrupts)
dxlAx dxlCom(&port);

// Serielle Eingabe Variablen
String _readString;         // Eingabestring vom Serial Monitor
bool _strComplete = false;  // Flag für vollständige String-Eingabe

// Dynamixel Servo IDs
int _id = 1;                // Hauptservo für Energieanzeige-Scheibe
int _idZeit = 2;            // Zweiter Servo für Zeitanzeige (deaktiviert)

/* ===== AUTO-KALIBRIERUNG VARIABLEN ===== */
bool kalibrierungAktiv = true;      // Flag für laufende Kalibrierung
unsigned long kalibrierungStart = 0; // Startzeit der Kalibrierung
const unsigned long KALIBRIERUNG_DAUER = 5000; // 5 Sekunden Kalibrierung
double maxLeerlaufWert = 0.0;       // Maximaler gemessener Leerlauf-Wert
double schwellwert = 0.27;          // Berechneter Schwellwert (Initial-Default)
const double SCHWELLWERT_OFFSET = 0.01; // Sicherheitsabstand über Leerlauf
int kalibrierungMessungen = 0;      // Anzahl Kalibrierungs-Messungen

/* ===== STROMSENSOR KONFIGURATION (ACS712) ===== */
const int Sensor = A0;      // Analoger Eingang für Stromsensor
const int VpA = 100;        // Millivolt pro Ampere (100 für 20A Modul, 66 für 30A Modul)
const int Nullpunkt = 2500; // Nullstrom-Spannung in mV (bei 2.5V)

// Stromsensor Messwerte
int sensorwert = 0;         // Roher ADC-Wert (0-1023)
double SensorSpannung = 0;  // Umgerechnete Spannung in mV
double Ampere = 0;          // Berechneter Strom in A
double AmpPos = 0;          // Positiver Stromwert (Gleichrichtung)
double AmpPosWeich = 0;     // Geglätteter positiver Stromwert
double AmpPosBisher = 0;    // Vorheriger Wert für Glättung

/* ===== STEUERUNGSLOGIK VARIABLEN ===== */
int Ausschlag = 614;        // Aktuelle Servo-Position (614 = oben, 0 = unten)
int killTimer = 0;          // Zähler für Inaktivitäts-Timeout
int running = 0;            // Status: 0=inaktiv, 1=aktive Stromerzeugung
double AmpCollected = 0;    // Akkumulierte Stromwerte seit Start

/* ===== TIMING KONFIGURATION ===== */
const double delayms = 500;     // Messintervall in ms
const double fulltime = 60000;  // Gesamtzeit für Timer in ms (1 Minute)
double timerms = 0;             // Verbleibende Timer-Zeit
int ZeitServoAusschlag = 0;     // Position für Zeit-Servo (deaktiviert)

// Funktionsprototypen
void printServoId(String msg);
void printDxlResult();
void printDxlError(unsigned short dxlError);

void setup() {
  // Serielle Kommunikation initialisieren (PC-Verbindung)
  Serial.begin(57600);
  while (!Serial) {
    ; // Warten auf Serial Port (nur bei nativen USB Ports nötig)
  }

  Serial.println("Stromvelo Controller gestartet!");

  // Dynamixel Kommunikation initialisieren
  dxlCom.begin(57600);

  // Startposition: Scheibe auf Nullstellung (oben) ausrichten
  Serial.println("Initialisiere Anzeigescheibe...");
  dxlCom.setMovingSpeed(_id, 150);  // Mittlere Geschwindigkeit für Initialisierung
  dxlCom.setGoalPosition(_id, 614); // Position 614 = Nullstellung (oben)
  
  /* 
   * Zeitanzeige-Servo deaktiviert um Strom zu sparen
   * dxlCom.setMovingSpeed(_idZeit, 150);
   * dxlCom.setGoalPosition(_idZeit, 0); 
   */

  // Auto-Kalibrierung starten
  Serial.println("=== AUTO-KALIBRIERUNG STARTET ===");
  Serial.println("Bitte 5 Sekunden NICHT in die Pedale treten!");
  Serial.println("Messe Leerlauf-Rauschen...");
  kalibrierungStart = millis();
  maxLeerlaufWert = 0.0;
  kalibrierungMessungen = 0;
}

/////////////////////////////////////////////////////////////////////////////////////

void loop() {
  /* ===== AUTO-KALIBRIERUNG PHASE ===== */
  if (kalibrierungAktiv) {
    // Prüfen ob Kalibrierungszeit abgelaufen
    if (millis() - kalibrierungStart >= KALIBRIERUNG_DAUER) {
      // Kalibrierung abschließen
      kalibrierungAktiv = false;
      schwellwert = maxLeerlaufWert + SCHWELLWERT_OFFSET;
      
      Serial.println("=== KALIBRIERUNG ABGESCHLOSSEN ===");
      Serial.print("Messungen: ");
      Serial.println(kalibrierungMessungen);
      Serial.print("Max. Leerlauf-Wert: ");
      Serial.print(maxLeerlaufWert, 4);
      Serial.println(" A");
      Serial.print("Berechneter Schwellwert: ");
      Serial.print(schwellwert, 4);
      Serial.println(" A");
      Serial.println("System bereit - Strommessung startet...");
      Serial.println("=====================================");
    } else {
      // Kalibrierungs-Messungen durchführen
      sensorwert = analogRead(Sensor);
      SensorSpannung = (sensorwert / 1024.0) * 5000;
      Ampere = ((SensorSpannung - Nullpunkt) / VpA);
      AmpPos = (Ampere < 0) ? 0 : Ampere;
      
      // Maximum der Leerlauf-Werte tracken
      if (AmpPos > maxLeerlaufWert) {
        maxLeerlaufWert = AmpPos;
      }
      
      kalibrierungMessungen++;
      
      // Progress-Anzeige alle Sekunde
      if (kalibrierungMessungen % (int)(1000/delayms) == 0) {
        unsigned long verbleibendeZeit = KALIBRIERUNG_DAUER - (millis() - kalibrierungStart);
        Serial.print("Kalibrierung... ");
        Serial.print(verbleibendeZeit / 1000);
        Serial.print("s verbleibend, aktueller Max: ");
        Serial.print(maxLeerlaufWert, 4);
        Serial.println(" A");
      }
      
      delay(delayms);
      return; // Hauptloop überspringen während Kalibrierung
    }
  }

  /* ===== STROMMESSUNG UND BERECHNUNG ===== */
  
  // ADC-Wert einlesen (0-1023 entspricht 0-5V)
  sensorwert = analogRead(Sensor);
  
  // Umrechnung: ADC-Wert -> Spannung in mV
  // 1024 ADC-Stufen entsprechen 5000mV
  SensorSpannung = (sensorwert / 1024.0) * 5000;
  
  // Stromberechnung: (Sensorspannung - Nullpunkt) / Empfindlichkeit
  // ACS712-20A: 100mV/A, Nullpunkt bei 2.5V
  // Positive Werte = Strom in eine Richtung, negative = andere Richtung
  Ampere = ((SensorSpannung - Nullpunkt) / VpA);

  // Debug-Ausgabe der Rohwerte
  Serial.print("Sensorwert:" );
  Serial.print(sensorwert); 
  Serial.print(",Sensorspannung:");
  Serial.print(SensorSpannung, 3);
  Serial.print(",Ampere:");
  Serial.print(Ampere, 3);

  /* ===== SIGNALAUFBEREITUNG ===== */
  
  // Gleichrichtung: Nur positive Stromwerte verwenden
  // (Richtung spielt keine Rolle für Energiemessung)
  AmpPosBisher = AmpPosWeich;
  AmpPos = (Ampere < 0) ? 0 : Ampere;  // Negative Werte auf 0 setzen
  
  Serial.print(",AmpPos:");
  Serial.print(AmpPos, 3);
  
  // Glättungsfilter: Gewichteter Durchschnitt zur Rauschunterdrückung
  // Neue Werte zählen 1/3, alte Werte 2/3 -> sanftere Übergänge
  AmpPosWeich = ((AmpPosBisher * 2) + AmpPos) / 3;
  
  Serial.print(",AmpPosWeich:");
  Serial.print(AmpPosWeich, 2);

  /* ===== SCHWELLENWERT-LOGIK UND ZUSTANDSSTEUERUNG ===== */
  
  // Aktivierung bei Überschreitung des kalibrierten Schwellenwerts
  // Schwellwert wird automatisch beim Start kalibriert
  if (AmpPosWeich > schwellwert && Ausschlag > 0) { 
    // Start einer neuen Energieproduktionsphase
    if (running == 0) {
      timerms = fulltime;  // Timer zurücksetzen
      Serial.println(" -> Energieproduktion gestartet!");
    }
    running = 1;
    killTimer = 0;
    AmpCollected += AmpPos;  // Stromwerte akkumulieren
    
  } else {
    // Inaktivitäts-Timer für automatisches Zurücksetzen
    if (running == 1) {
      killTimer++;
      
      // Nach 10 Messzyklen ohne Strom (5 Sekunden) zurücksetzen
      if (killTimer > 10) {
        Serial.println(" -> Energieproduktion beendet - Reset");
        running = 0;
        killTimer = 0;
        AmpCollected = 0;
        
        // Scheibe langsam zurück zur Startposition
        dxlCom.setMovingSpeed(_id, 100);
        dxlCom.setGoalPosition(_id, 614);  // Zurück zur Nullstellung
        timerms = 0;
        
        // 4 Sekunden warten bis Servo-Bewegung abgeschlossen
        delay(4000);
        
        /* Zeitanzeige zurücksetzen (deaktiviert)
         * dxlCom.setMovingSpeed(_idZeit, 100);
         * dxlCom.setGoalPosition(_idZeit, 0);
         */
      }
    }
  }

  // Status-Debug-Ausgabe
  Serial.print(",running:");
  Serial.print(running);
  Serial.print(",killTimer:");
  Serial.print(killTimer);
  Serial.print(",schwellwert:");
  Serial.print(schwellwert, 4);
  Serial.print(",timerms:");
  Serial.print(timerms);
  Serial.print(",AmpCollected:");
  Serial.print(AmpCollected, 2);

  /* ===== SERVO-POSITIONSBERECHNUNG ===== */
  
  // Mapping: Gesammelte Energie -> Servo-Position
  // - Maximum: 20A entspricht 180° Bewegung
  // - Servo-Range: 0-1023 für 300°, davon 614 für 180°
  // - Position 614 = Start (oben), Position 0 = Ziel (unten)
  // - Rundung auf 2er-Schritte für sanftere Bewegung
  Ausschlag = 614 - round(AmpCollected / 20 * 614 / 2) * 2;
  
  // Begrenzung: Nicht unter Null (Zielposition erreicht)
  if (Ausschlag < 0) {
    Ausschlag = 0;
  }

  Serial.print(",Servo:");
  Serial.print(Ausschlag);

  // Servo-Bewegung mit reduzierter Geschwindigkeit für sanfte Bewegung
  dxlCom.setMovingSpeed(_id, 50);
  dxlCom.setGoalPosition(_id, Ausschlag);

  /* ===== TIMING UND ZEITANZEIGE ===== */
  
  // Haupt-Loop-Timing
  delay(delayms);
  
  // Timer herunterzählen
  timerms -= delayms;
  if (timerms < 0) {
    timerms = 0;
  }
  
  // Zeitanzeige-Servo Position berechnen (deaktiviert)
  ZeitServoAusschlag = round(timerms / fulltime * 1023);
  
  /* Zeitanzeige-Servo Steuerung (deaktiviert um Strom zu sparen)
   * dxlCom.setMovingSpeed(_idZeit, 25);
   * dxlCom.setGoalPosition(_idZeit, ZeitServoAusschlag);
   */

  Serial.print(",ZeitServoAusschlag:");
  Serial.println(ZeitServoAusschlag);

  /* ===== SERIELLE KOMMANDO-VERARBEITUNG ===== */
  
  // Zeichen aus Serial Buffer lesen
  while (Serial.available()) {
    char inputChar = Serial.read();
    _readString += inputChar;

    if (inputChar == '\n')
      _strComplete = true;
  }

  // Vollständige Kommandos verarbeiten
  if (_strComplete) {
    _strComplete = false;
    
    // Servo ID ändern
    if (_readString.startsWith("ID")) {
       _readString.remove(0, 2);
       _id = _readString.toInt();
       printServoId("Kommuniziere mit");
       Serial.println(_id);
    }
    // Ping-Test
    else if (_readString.startsWith("ping")) {
      printServoId("Ping");
      dxlCom.ping(_id);
      printDxlResult();
    }
    // Action-Befehl ausführen
    else if (_readString.startsWith("action")) {
      dxlCom.isRegistered(_id);
      while(!dxlCom.dxlDataReady());
      printDxlError(dxlCom.readDxlError());
      if (dxlCom.readDxlResult()) {
        printServoId("Führe REG-Befehl aus für");
        while(dxlCom.isBusy());
        dxlCom.action(_id);
        printDxlResult();
      } else {
        printServoId("Kein REG-Befehl für");
        Serial.println();
      }
    }
    // Servo-Neustart (nicht von MX unterstützt)
    else if (_readString.startsWith("reboot")) {
      printServoId("Neustart (nicht von MX unterstützt)");
      dxlCom.reboot(_id);
      printDxlResult();
    }
    // Modellnummer abfragen
    else if (_readString.startsWith("model")) {
      printServoId("Modellnummer von");
      dxlCom.readModelNumber(_id);
      printDxlResult();
    }
    // Firmware-Version abfragen
    else if (_readString.startsWith("firmware")) {
      printServoId("Firmware-Version von");
      dxlCom.readFirmware(_id);
      printDxlResult();
    }
    // Neue Servo-ID setzen
    else if (_readString.startsWith("setID")) {
      _readString.remove(0, 5);
      int newID = _readString.toInt();
      printServoId("Setze ID von");
      Serial.print(_id);
      Serial.print(" auf ");
      Serial.print(newID);
      Serial.print(" : ");
      dxlCom.setId(_id, newID);
      printDxlResult();
    }
    // LED-Status ändern
    else if (_readString.startsWith("led")) {
      _readString.remove(0, 3);
      bool Status = _readString.toInt();
      printServoId("Ändere LED-Status von ");
      dxlCom.setLedEnable(_id, Status);
      printDxlResult();
    }
    // Direkte Positionierung
    else if (_readString.startsWith("move")) {
      _readString.remove(0, 4);
      unsigned short Position = _readString.toInt();
      printServoId("Bewege ");
      dxlCom.setGoalPosition(_id, Position);
      printDxlResult();

      // Bewegung überwachen bis Ziel erreicht
      bool isMoving = true;
      while (isMoving) {
        unsigned short error = DXL_ERR_SUCCESS;
        while(dxlCom.isBusy());
        
        // Aktuelle Position abfragen
        dxlCom.readPresentPosition(_id);
        Serial.print("Pos : ");
        while(!dxlCom.dxlDataReady());
        error = dxlCom.readDxlError();
        if(error != DXL_ERR_SUCCESS)
          printDxlError(error);
        Serial.println(dxlCom.readDxlResult());
        
        // Bewegungsstatus prüfen
        while(dxlCom.isBusy());
        dxlCom.isMoving(_id);
        while(!dxlCom.dxlDataReady());
        error = dxlCom.readDxlError();
        if(error != DXL_ERR_SUCCESS)
          printDxlError(error);
        isMoving = dxlCom.readDxlResult();
      }
    }
    // Geschwindigkeit setzen
    else if (_readString.startsWith("speed")) {
      _readString.remove(0, 5);
      unsigned short Speed = _readString.toInt();
      printServoId("Setze Geschwindigkeit von ");
      dxlCom.setMovingSpeed(_id, Speed);
      printDxlResult();
    }
    // Drehmoment setzen
    else if (_readString.startsWith("torque")) {
      _readString.remove(0, 6);
      unsigned short torque = _readString.toInt();
      printServoId("Setze Drehmoment von ");
      dxlCom.setTorqueLimit(_id, torque);
      printDxlResult();
    }
    // Spannung auslesen
    else if (_readString.startsWith("voltage")) {
      printServoId("Spannung (durch 10 teilen) von");
      dxlCom.readVoltage(_id);
      printDxlResult();
    }
    // Temperatur auslesen
    else if (_readString.startsWith("temperature")) {
      printServoId("Temperatur von");
      dxlCom.readTemperature(_id);
      printDxlResult();
    }
    // Registrierte Bewegung vorbereiten
    else if (_readString.startsWith("regmove")) {
      _readString.remove(0, 7);
      unsigned short Position = _readString.toInt();
      printServoId("Schreibe Befehl (mit 'action' ausführen) in REG-Register von ");
      dxlCom.sendDxlRegData(_id, DXL_ADD_GOAL_POSITION, (const byte*) &Position, 2);
      printDxlResult();
    }

    _readString = ""; // Buffer für nächste Eingabe leeren
  }
}

/////////////////////////////////////////////////////////////////////////////////////
/* ===== HILFSFUNKTIONEN ===== */

void printDxlResult() {
   while(!dxlCom.dxlDataReady());        // Warten auf Servo-Antwort
   printDxlError(dxlCom.readDxlError());
   Serial.println(dxlCom.readDxlResult());
}

void printServoId(String msg) {
  Serial.print(msg);
  Serial.print(" Servo ID ");
  Serial.print(_id);
  Serial.print(" - ");
}

void printDxlError(unsigned short dxlError) {
  // Fehlerauswertung nach jeder Operation
  if(dxlError == DXL_ERR_SUCCESS)
    Serial.println("OK");
  else {
    if (dxlError & DXL_ERR_VOLTAGE)
      Serial.print("Spannung außerhalb Bereich-");
    if (dxlError & DXL_ERR_ANGLE)
      Serial.print("Winkel außerhalb Bereich-");
    if (dxlError & DXL_ERR_OVERHEATING)
      Serial.print("Überhitzung-");
    if (dxlError & DXL_ERR_RANGE)
      Serial.print("Befehl außerhalb Bereich-");
    if (dxlError & DXL_ERR_TX_CHECKSUM)
      Serial.print("Tx CRC ungültig-");
    if (dxlError & DXL_ERR_OVERLOAD )
      Serial.print("Überlastung-");
    if (dxlError & DXL_ERR_INSTRUCTION )
      Serial.print("Unbekannter Befehl-");
    if (dxlError & DXL_ERR_TX_FAIL )
      Serial.print("Tx kein Header-");
    if (dxlError & DXL_ERR_RX_FAIL )
      Serial.print("Rx kein Header-");
    if (dxlError & DXL_ERR_TX_ERROR  )
      Serial.print("Tx Fehler-");
    if (dxlError & DXL_ERR_RX_LENGTH   )
      Serial.print("Rx Länge ungültig-");
    if (dxlError & DXL_ERR_RX_TIMEOUT)
      Serial.print("Timeout-");
    if (dxlError & DXL_ERR_RX_CORRUPT)
      Serial.print("Rx CRC ungültig-");
    if (dxlError & DXL_ERR_ID )
      Serial.print("Falsche ID geantwortet-");
    Serial.println();
  }
}