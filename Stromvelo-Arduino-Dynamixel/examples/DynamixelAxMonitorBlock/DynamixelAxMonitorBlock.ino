/*
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
#include <SoftHalfDuplexSerial.h>
#include <DynamixelAx.h>

softHalfDuplexSerial port(8); // data pin 8
dxlAx dxlCom(&port);

String _readString;         // Input string from serial monitor
bool _strComplete = false;
int _id = 1;                // Default Dynamixel servo ID


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

}

/////////////////////////////////////////////////////////////////////////////////////

void loop()
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
