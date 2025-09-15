/*
Stromvelo-Arduino-Dynamixel.ino

Dieser für einen Arduino Mega mit angeschlossenem Dynamixel AX-12 Servo geschriebener
Code, dreht eine Weltscheibe, wenn ein Strom von einem Generator, der durch ein
Fahrrad angetrieben gemessen wird. 
Der Strom wird mit einem Stromstärkesensor ACS712 gemessen.

Zur Ansteuerung der Servos habe ich kein Dynamixel-Shield zur Verfügung.
Aus diesem Grund steuere ich den Servo direkt über die Daten-Schnittstelle
des Arduino an. Die dafür eingesetzte Technik habe ich von 
DynamixelMxMonitorBlock.ino welches von Akira geschrieben wurde übernommen.
Untenstehend ist der Originalheader davon.

 *****************************************************************************
DynamixelMxMonitorBlock.ino
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
 Decription:
 This library implement all the required operation to drive Dynamixel servo,
 Please visit http://support.robotis.com/en/product/dynamixel/dxl_communication.htm to understand Dynamixel communication protocol

 Hardware:
  - This example use softHalfDuplexSerial library, check that the connected data pin support change interrupt
  - Pin 8 is used in this example to communicate with dynamixel servos (only one wire)
  - Check that the board is properly grounded with dynamixel power supply.
  - Please check the servo baudrate, it has been set to 57 600bps here.
  - Take care that the defaut servo ID is 1.
  In this example, we will wait after each Dynamixel command the answer of the servo.
  The code is simplier than in non blocking implemantation, but it will block any other code (except interrupt) during this time.
*/
#include "src\SoftHalfDuplexSerial.h"
#include "src\DynamixelAx.h"

softHalfDuplexSerial port(50); // Defaut 8 arduino uno / but on mega 50 because needs to support pin change interrupts
dxlAx dxlCom(&port);

String _readString;         // Input string from serial monitor
bool _strComplete = false;
int _id = 1;                // Default Dynamixel servo ID für Scheibe
int _idZeit = 2;            // Zweiter Dynamixel für Zeitanzeige auf der linken Seite, deaktiviert


/* Basiswerte für Stromstärkesensor ACS712 */
int Sensor = A0; // Der Stromstärkesensor wird am Pin A0 (Analog "0") angeschlossen.
int VpA = 100; // Millivolt pro Ampere (100 für 20A Modul und 66 für 30A Modul)
int sensorwert= 0;
/* Bei einer Signalspannung des Sensors von 0,5 Volt beträgt die Stromstärke -20 Ampere */
/* und bei 4,5 Volt beträgt die Stromstärke 20 Ampere (je nach Verkabelung positiv oder negativ). */
/* Der Nullpunkt ist also bei 2,5 Volt */
int Nullpunkt = 2500; // Spannung in mV bei dem keine Stromstärke vorhanden ist
double SensorSpannung = 0;
double Ampere = 0;
double AmpPos = 0;
double AmpPosWeich = 0;
double AmpPosBisher = 0;
int Ausschlag = 614;
int killTimer = 0;
int running = 0;
double AmpCollected = 0;
double delayms = 500; // zwischen den Messungen eine halbe Sekunde warten
double fulltime = 60000;
double timerms = 0;
int ZeitServoAusschlag = 0;

void printServoId(String msg);
void printDxlResult();
void printDxlError(unsigned short dxlError);

void setup() {

  // Open serial communications and wait for port to open (PC communication)
  Serial.begin(57600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("Starting COM!");

  dxlCom.begin(57600);

  // Zum Start wird die Weltscheibe korrekt ausgerichtet: Position 614 = oben / 0 = unten
  dxlCom.setMovingSpeed(_id, 150); // set velocity to 200(range:0-300) in Servo mode
  dxlCom.setGoalPosition(_id, 614);
  
  /* Deaktivere Zeitanzeige um Strom zu sparen
  dxlCom.setMovingSpeed(_idZeit, 150); // set velocity to 200(range:0-300) in Servo mode
  dxlCom.setGoalPosition(_idZeit, 0); 
  */

}

/////////////////////////////////////////////////////////////////////////////////////

void loop()
{
    /* Stromstärkesensor einlesen */ 
  sensorwert = analogRead(Sensor);
  SensorSpannung = (sensorwert / 1024.0) * 5000; // Hier wird der Messwert in den Spannungswert am Sensor umgewandelt.
  Ampere = ((SensorSpannung - Nullpunkt) / VpA); // Im zweiten Schritt wird hier die Stromstärke berechnet.

  // Ausgabe der Ergebnisse am Seriellen Monitor
  Serial.print("Sensorwert:" ); // Ausgabe des reinen Sensorwertes
  Serial.print(sensorwert); 
  
  Serial.print(",Sensorspannung:"); // Zeigt die Sensorspannung an
  Serial.print(SensorSpannung,3); // Die "3" hinter dem Komma erzeugt drei Nachkommastellen
  Serial.print(",Ampere:"); // shows the voltage measured 
  Serial.print(Ampere,3); // Die "3" hinter dem Komma erzeugt drei Nachkommastellen
  

  // Umrechnung Ampere immer positiv, egal wie die Kabel angeschlossen sind 
  AmpPosBisher = AmpPosWeich;
  AmpPos = Ampere;
  if (AmpPos<0) {
    AmpPos = 0;
  }
  
  Serial.print(",AmpPos:"); // immer Positiv 
  Serial.print(AmpPos,3); // Die "3" hinter dem Komma erzeugt drei Nachkommastellen
  
  // Wegen recht grossen Schwankungen wird der Wert mit dem vorherigen Wert 
  // verrechnet umd einen weicheren (geglätteten) Ampere-Wert zu erhalten
  AmpPosWeich = ((AmpPosBisher)*2 + AmpPos) / 3; // Der neue Wert zählt nur 1/3el 
  Serial.print(",AmpPosWeich:"); // immer Positiv 
  Serial.print(AmpPosWeich,2); // Die "2" hinter dem Komma erzeugt zwei Nachkommastellen

  // Hier wird der Schwellwert gesetzt, bei welcher Ampere-Anzahl
  // der Wert überhaupt verwendet werden soll.
  // Dies ist notwenig, da viel Noise bei der Messung entsteht

  // grm 17.5.2025 / Bisher war 0.24 ein guter Wert
  // Ich setze diesen runter um einen Dauerbetrieb zu simulieren
  // MUSS WIEDER ZURUECKGESETZT WERDEN!
  //if (AmpPosWeich > 0.14 && Ausschlag > 0) { 
  if (AmpPosWeich > 0.27 && Ausschlag > 0) { 
    if (running==0) {
      timerms = fulltime;
    }
    running = 1;
    killTimer = 0;
    AmpCollected = AmpCollected + AmpPos;
  } else {
    if (running==1) {
      killTimer = killTimer + 1;
      // Falls 10 Delay (also 5 Sec) kein neuer Impuls, wird die Scheibe zurückgesetzt
      // grm 17.5.2025 / Zum Lasttest, verändere ich die Bedingung
      // MUSS WIEDER ZURUECKGESETZT WERDEN!
      //if (Ausschlag == 0) {
      if (killTimer > 10) {
        running = 0;
        killTimer = 0;
        AmpCollected = 0;
        dxlCom.setMovingSpeed(_id, 100); // set velocity to 200(range:0-300) in Servo mode
        dxlCom.setGoalPosition(_id, 614);
        timerms = 0;
        // 4 Sekunden warten, damit Scheibe fertig gedreht ist bevor es weitergeht
        delay(4000);
        /* Deaktivere Zeitanzeige um Strom zu sparen 
        dxlCom.setMovingSpeed(_idZeit, 100); // set velocity to 200(range:0-300) in Servo mode
        dxlCom.setGoalPosition(_idZeit, 0);    
        */    
      }
    }
  }

  Serial.print(",running:"); 
  Serial.print(running); 

  Serial.print(",killTimer:"); 
  Serial.print(killTimer); 

  Serial.print(",timerms:"); 
  Serial.print(timerms); 

  Serial.print(",AmpCollected:"); 
  Serial.print(AmpCollected, 2); 


  // Max. Ampere ist 20A und 180 Grad Ausschlag. 
  // Beim Servo ist 0 das min und 1023 das max bei 300 Grad Bewegung
  // Für 180 Grad Bewegung ist das max bei 614
  // Damit er nicht zu sehr hüpft wird es in 5er-Schritten gerundet
  Ausschlag = 614 - round(AmpCollected / 20 * 614 / 2) * 2; 
  // Wenn er am Ziel (Ausschlag=0) angelangt ist, bleibt er stehen
  // auch wenn neue Werte dazukommen und so ins Minus gehen
  if (Ausschlag < 0) {
    Ausschlag = 0;
  }


  Serial.print(",Servo:"); // shows the voltage measured 
  Serial.print(Ausschlag); 

  dxlCom.setMovingSpeed(_id, 50); // set velocity to 200(range:0-300) in Servo mode
  dxlCom.setGoalPosition(_id, Ausschlag);

  delay(delayms); 
  timerms = timerms - delayms;
  if (timerms<0) {
    timerms = 0;
  }
  ZeitServoAusschlag = round(timerms / fulltime * 1023); 
  /* Deaktiviere Zeitanzeige um Strom zu sparen 
  dxlCom.setMovingSpeed(_idZeit, 25); // set velocity to 200(range:0-300) in Servo mode
  dxlCom.setGoalPosition(_idZeit, ZeitServoAusschlag);
  */

  Serial.print(",ZeitServoAusschlag:"); // shows the voltage measured 
  Serial.println(ZeitServoAusschlag); 


  while (Serial.available())
  {
    char inputChar = Serial.read();  //gets one byte from serial buffer
    _readString += inputChar; //makes the string readString

    if (inputChar=='\n')
      _strComplete = true;

  }

  if (_strComplete)
  {
    _strComplete = false;
    if (_readString.startsWith("ID"))
    {
       _readString.remove(0, 2);
       _id = _readString.toInt();  //convert readString into a number
       printServoId("Communicating with");
       Serial.println(_id);
    }
    else if (_readString.startsWith("ping"))
    {
      printServoId("Ping");
      dxlCom.ping(_id);
      printDxlResult();
    }
    else if (_readString.startsWith("action"))
    {
      dxlCom.isRegistered(_id);
      while(!dxlCom.dxlDataReady());          // waiting the answer of servo
      printDxlError(dxlCom.readDxlError());
      if (dxlCom.readDxlResult())               // if it is registred
      {
        printServoId("Execute reg command in");
        while(dxlCom.isBusy()); // waiting the status return delay time
        dxlCom.action(_id);
        printDxlResult();
      }
      else
      {
        printServoId("No reg command in");
        Serial.println();
      }
    }
    else if (_readString.startsWith("reboot"))
    {
      printServoId("Reboot (not supported by MX)");
      dxlCom.reboot(_id);
      printDxlResult();
    }
    else if (_readString.startsWith("model"))
    {
      printServoId("Model number of");
      dxlCom.readModelNumber(_id);
      printDxlResult();
    }
    else if (_readString.startsWith("firmware"))
    {
      printServoId("Firmware number of");
      dxlCom.readFirmware(_id);
      printDxlResult();
    }
    else if (_readString.startsWith("setID"))
    {
      _readString.remove(0, 5);
      int newID = _readString.toInt();  //convert readString into a number
      printServoId("Setting");
      Serial.print(_id);
      Serial.print(" to ");
      Serial.print(newID);
      Serial.print(" : ");
      dxlCom.setId(_id, newID);
      printDxlResult();
    }
    else if (_readString.startsWith("led"))
    {
      _readString.remove(0, 3);
      bool Status = _readString.toInt();  //convert readString into a number
      printServoId("Changing led status of ");
      dxlCom.setLedEnable(_id,Status);
      printDxlResult();
    }
    else if (_readString.startsWith("move"))
    {
      _readString.remove(0, 4);
      unsigned short Position = _readString.toInt();  //convert readString into a number
      printServoId("Moving ");
      dxlCom.setGoalPosition(_id,Position);
      printDxlResult();

      bool isMoving = true;

      while (isMoving)
      {
        unsigned short error = DXL_ERR_SUCCESS;
        while(dxlCom.isBusy()); // waiting the status return delay time
        dxlCom.readPresentPosition(_id);
        Serial.print("Pos : ");
        while(!dxlCom.dxlDataReady());        // waiting the answer of servo
        error = dxlCom.readDxlError();
        if(error!=DXL_ERR_SUCCESS) // readDxlResult should always be called before readDxlData
          printDxlError(error);
        Serial.println(dxlCom.readDxlResult());
        while(dxlCom.isBusy()); // waiting the status return delay time (for testing if it is moving)
        dxlCom.isMoving(_id);
        while(!dxlCom.dxlDataReady());        // waiting the answer of servo
        error = dxlCom.readDxlError();
        if(error!=DXL_ERR_SUCCESS) // readDxlResult should always be called before readDxlData
          printDxlError(error);
        isMoving = dxlCom.readDxlResult();
      }

    }
    else if (_readString.startsWith("speed"))
    {
      _readString.remove(0, 5);
      unsigned short Speed = _readString.toInt();  //convert readString into a number
      printServoId("Set speed of ");
      dxlCom.setMovingSpeed(_id,Speed);
      printDxlResult();
    }
    else if (_readString.startsWith("torque"))
    {
      _readString.remove(0, 6);
      unsigned short torque = _readString.toInt();  //convert readString into a number
      printServoId("Set torque of ");
      dxlCom.setTorqueLimit(_id,torque);
      printDxlResult();
    }
    else if (_readString.startsWith("voltage"))
    {
      printServoId("Voltage (to be divided by 10) of");
      dxlCom.readVoltage(_id);
      printDxlResult();
    }
    else if (_readString.startsWith("temperature"))
    {
      printServoId("Temperature of");
      dxlCom.readTemperature(_id);
      printDxlResult();
    }
    else if (_readString.startsWith("regmove"))
    {
      _readString.remove(0, 7);
      unsigned short Position = _readString.toInt();  //convert readString into a number
      printServoId("Write command (type 'action' to execute) in REG register of ");
      dxlCom.sendDxlRegData(_id, DXL_ADD_GOAL_POSITION, (const byte*) &Position, 2 );
      printDxlResult();
    }

    _readString=""; //empty for next input
  }

}

/////////////////////////////////////////////////////////////////////////////////////

void printDxlResult()
{
   while(!dxlCom.dxlDataReady());        // waiting the answer of servo
   printDxlError(dxlCom.readDxlError());
   Serial.println(dxlCom.readDxlResult());
}

void printServoId(String msg)
{
  Serial.print(msg);
  Serial.print(" servo ID ");
  Serial.print(_id);
  Serial.print(" - ");
}

void printDxlError(unsigned short dxlError)
{
  // after any operation error can be retrieve using dx::readDxlResult() (i.e. after read or write operation)
  if(dxlError == DXL_ERR_SUCCESS)
    Serial.println("OK");
  else
  {
    if (dxlError & DXL_ERR_VOLTAGE)
      Serial.print("voltage out of range-");
    if (dxlError & DXL_ERR_ANGLE)
      Serial.print("angle out of range-");
    if (dxlError & DXL_ERR_OVERHEATING)
      Serial.print("overheating-");
    if (dxlError & DXL_ERR_RANGE)
      Serial.print("cmd out of range-");
    if (dxlError & DXL_ERR_TX_CHECKSUM)
      Serial.print("Tx CRC invalid-");
    if (dxlError & DXL_ERR_OVERLOAD )
      Serial.print("overload-");
    if (dxlError & DXL_ERR_INSTRUCTION )
      Serial.print("undefined instruction-");
    if (dxlError & DXL_ERR_TX_FAIL )
      Serial.print("Tx No header-");
    if (dxlError & DXL_ERR_RX_FAIL )
      Serial.print("Rx No header-");
    if (dxlError & DXL_ERR_TX_ERROR  )
      Serial.print("Tx error-");
    if (dxlError & DXL_ERR_RX_LENGTH   )
      Serial.print("Rx length invalid-");  // Not implemented yet
    if (dxlError & DXL_ERR_RX_TIMEOUT)
      Serial.print("timeout-");
    if (dxlError & DXL_ERR_RX_CORRUPT)
      Serial.print("Rx CRC invalid-");
    if (dxlError & DXL_ERR_ID )
      Serial.print("Wrong ID answered-"); // ?? Hardware issue
    Serial.println();
  }
}
