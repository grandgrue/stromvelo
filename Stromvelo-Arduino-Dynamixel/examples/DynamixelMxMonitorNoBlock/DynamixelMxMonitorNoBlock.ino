/*
DynamixelMxMonitorNoBlock.ino
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
  - This example use Serial1 port, so an Arduino with this port is required. UART Tx & Rx pin should be respectively connected to MAX485 DI & RO pin.
  - Pin 53 is used to switch between Tx & Rx mode. It should be connected to !RE & DE simultaneously.
  - Please check the servo baudrate, it has been set to 400 000bps here.
  - Take care that the defaut servo ID is 2.
*/
#include <HardHalfDuplexSerial.h>
#include <DynamixelMx.h>

#define TRACKING_IDLE     0
#define TRACKING_MOVING   1
#define TRACKING_POSITION 2

dxlMx dxlCom(&hdSerial1); //  using Serial1 (Tx 18, Rx 19)

String _readString;         // Input string from serial monitor
bool _strComplete = false;
int _id = 2;
bool _trackDxlPosition = false;
byte _trackingState = TRACKING_IDLE;
bool _action = false;

void printServoId(String msg);
void processUserInput();
void processDxlData();
void trackPositon();


void setup() {

  // Open serial communications and wait for port to open (PC communication)
  Serial.begin(57600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("Starting COM!");

  pinMode(19,INPUT_PULLUP);  // To be replaced by hardware pull up

  // On Arduino Due, only the corresponding USART RTS pin is used to drive the controller
#if !defined  (__SAM3X8E__)
  hdSerial1.setDirPin(53);   // dirPin pin 53,
#endif

  dxlCom.begin(400000);

}

/////////////////////////////////////////////////////////////////////////////////////

void loop()
{
  processUserInput();
  processDxlData();
}

/////////////////////////////////////////////////////////////////////////////////////

void processDxlData()
{
  if (dxlCom.dxlDataReady())
  {
      if(_trackDxlPosition)
        trackPosition();
      else if (_action)
        action();
      else
      {
        //unsigned short dxlError = dxlCom.readDxlError();
        if(dxlCom.readDxlError() == DXL_ERR_SUCCESS)
        {
          Serial.print("Rx Status OK - ");
          Serial.println(dxlCom.readDxlResult());
        }
        else
          printDxlError(dxlCom.readDxlError());
      }


  } // dxlCom.dxlDataReady()
}

void trackPosition()
{
   if(dxlCom.isBusy())
    return;

   //unsigned short dxlError = dxlCom.readDxlError();
   if(dxlCom.readDxlError() == DXL_ERR_SUCCESS)
   {
      switch (_trackingState)
      {
              case TRACKING_IDLE:
                dxlCom.isMoving(_id);
                _trackingState = TRACKING_MOVING;
              break;

              case TRACKING_MOVING:
                if(dxlCom.readDxlResult())            // dxl is moving
                {
                  dxlCom.readPresentPosition(_id);
                  _trackingState = TRACKING_POSITION;
                }
                else
                {
                  _trackDxlPosition = false;
                  _trackingState = TRACKING_IDLE;
                }
              break;

              case TRACKING_POSITION:
                Serial.print("Pos : ");  Serial.println(dxlCom.readDxlResult());
                dxlCom.isMoving(_id);
                _trackingState = TRACKING_MOVING;
              break;
      } //switch (_trackingState)
   }    // if(dxlError == DXL_ERR_SUCCESS)
   else
      printDxlError(dxlCom.readDxlError());
}

void action()
{
  if(dxlCom.isBusy())
    return;

  if(dxlCom.readDxlError() == DXL_ERR_SUCCESS)
    if(dxlCom.readDxlResult())            // dxl is registred
    {
      printServoId("Execute reg command in");
      dxlCom.action(_id);
      _action = false;                  // no need to check the result of action
    }
    else
    {
      printServoId("No reg command in");
      Serial.println();
    }
  else
    printDxlError(dxlCom.readDxlError());
}

void processUserInput()
{
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
    }
    else if (_readString.startsWith("action"))
    {
      _action = dxlCom.isRegistered(_id);  // return true if transmit succeed
    }
    else if (_readString.startsWith("reboot"))
    {
      printServoId("Reboot (not supported by MX)");
      dxlCom.reboot(_id);
    }
    else if (_readString.startsWith("model"))
    {
      printServoId("Model number of");
      dxlCom.readModelNumber(_id);
    }
    else if (_readString.startsWith("firmware"))
    {
      printServoId("Firmware number of");
      dxlCom.readFirmware(_id);
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
    }
    else if (_readString.startsWith("led"))
    {
      _readString.remove(0, 3);
      bool Status = _readString.toInt();  //convert readString into a number
      printServoId("Changing led status of ");
      dxlCom.setLedEnable(_id,Status);
    }
    else if (_readString.startsWith("move"))
    {
      _readString.remove(0, 4);
      unsigned short Position = _readString.toInt();  //convert readString into a number
      printServoId("Moving ");
      dxlCom.setGoalPosition(_id,Position);
      _trackDxlPosition = true;
    }
    else if (_readString.startsWith("speed"))
    {
      _readString.remove(0, 5);
      unsigned short Speed = _readString.toInt();  //convert readString into a number
      printServoId("Set speed of ");
      dxlCom.setMovingSpeed(_id,Speed);
    }
    else if (_readString.startsWith("torque"))
    {
      _readString.remove(0, 6);
      unsigned short torque = _readString.toInt();  //convert readString into a number
      printServoId("Set torque of ");
      dxlCom.setTorqueLimit(_id,torque);
    }
    else if (_readString.startsWith("voltage"))
    {
      printServoId("Voltage (to be divided by 10) of");
      dxlCom.readVoltage(_id);
    }
    else if (_readString.startsWith("temperature"))
    {
      printServoId("Temperature of");
      dxlCom.readTemperature(_id);
    }
    else if (_readString.startsWith("regmove"))
    {
      _readString.remove(0, 7);
      unsigned short Position = _readString.toInt();  //convert readString into a number
      printServoId("Write command (type 'action' to execute) in REG register of ");
      dxlCom.sendDxlRegData(_id, DXL_ADD_GOAL_POSITION, (const byte*) &Position, 2 );
    }

    _readString=""; //empty for next input
  }
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
