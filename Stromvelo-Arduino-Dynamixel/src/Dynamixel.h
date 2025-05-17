/*
Dynamixel.h
Written by Akira

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

 Limitations:
  - Baudrate is limited 57 600bps ~ 115 200bps for 16Mhz Arduino board with softHalfDuplexSerial, and ~ 400 000bps for 16Mhz Arduino board with hardHalfDuplexSerial
  - This library is non blocking, i.e. when a write or a read command occured, it will NOT wait the answer of the servo
*/
#ifndef Dynamixel_h
#define Dynamixel_h

#include "Arduino.h"
#include "HardHalfDuplexSerial.h"
#include "DynamixelConfig.h"


/******************************************************************************
* Definitions
******************************************************************************/

class dxl
{
private:
  halfDuplexSerial* _port;               // Serial port to be used to communicate with dynamixel
  unsigned short  _error;                // Store the last error
  byte _nByteToBeRead;                   // Used to check when Rx packet is complete
  byte _currentId;                       // Used to check if the rx packet comes from the correct servo
  unsigned long _timeout;                // Store Rx timeout (default 1000ms)
  unsigned long _currentTimeout;         // To be compared with millis();
  unsigned long _returnDelayTime;        // Store the return time of Dynamixel servo (dfault 1000 µs)
  unsigned long _currentReturnDelayTime; // this variable assume that dxlDataReady();is called periodically
  unsigned short _dxlResult;             // Dxl read will be stored here

  // This is the only function to write on the serial port
  bool writeRaw( const byte *sentence, const byte nByteToBeWritten);
  unsigned short readDxlData(); 		      // the only function to read on serial port

public:
  
  // public methods
  dxl(halfDuplexSerial* port);
  ~dxl();
  void begin(const unsigned long speed);

  bool ping(const byte  ID);    // ping the servo
  bool action(const byte  ID);  // Excecute action written in REG_WRITE servo register. A status packet is receive, although the documentation don't mention it
  bool reset(const byte  ID);   // Reset the servo to factory default setting
  bool reboot(const byte  ID);  // Reboot the servo (not supported by mx servos)

  //
  //  EEPROM commands
  //

  bool readModelNumber(const byte ID);
  bool readFirmware(const byte ID);

  bool setId(const byte  ID, const byte newID);
  bool readId(const byte ID);    // Is this really usefull ?
  
  bool setBaudRate(const byte  ID, const byte baudRate);
  bool readBaudRate(const byte ID); 
  
  bool setReturnDelayTime(const byte  ID, const byte returnDelayTime);
  bool readReturnDelayTime(const byte ID);

  bool setCWAngleLimit(const byte  ID, const unsigned short angle);
  bool setCCWAngleLimit(const byte  ID, const unsigned short angle);

  bool readCWAngleLimit(const byte ID);
  bool readCCWAngleLimit(const byte ID);

  bool setMaxTemperature(const byte  ID, const byte maxTemperature);
  bool readMaxTemperature(const byte ID);

  bool setMinVoltage(const byte  ID, const byte minVoltage);
  bool readMinVoltage(const byte ID);

  bool setMaxVoltage(const byte  ID, const byte maxVoltage);
  bool readMaxVoltage(const byte ID);

  bool setMaxTorque(const byte  ID, const unsigned short maxTorque);
  bool readMaxTorque(const byte ID);

  bool setStatusReturnLevel(const byte  ID, const byte Status);
  bool readStatusReturnLevel(const byte ID);
  
  bool setAlarmLed(const byte  ID, const byte Status);
  bool readAlarmLed(const byte ID);

  bool setAlarmShutdown(const byte  ID, const byte Status);
  bool readAlarmShutdown(const byte ID);

  //
  //  RAM commands
  //
  bool setTorqueEnable(const byte  ID, const bool Status);
  bool readTorqueEnable(const byte ID);

  bool setLedEnable(const byte  ID, const bool Status);
  bool readLedEnable(const byte  ID);

  bool setGoalPosition(const byte ID, const short Position);
  bool setGoalPositionAtSpeed(const byte ID, const short Position, const short Speed);
  bool setMovingSpeed(const byte ID, const short Speed);
  bool setTorqueLimit(const byte ID, const short torque);

  bool readPresentPosition(const byte ID);
  bool readPresentSpeed(const byte ID);
  bool readPresentLoad(const byte ID);

  bool readVoltage(const byte ID);
  bool readTemperature(const byte ID);
  bool isRegistered(const byte ID);
  bool isMoving(const byte ID);

  bool setEEPROMLock(const byte ID, const bool lock);
  bool isEEPROMLock(const byte ID);

  bool setPunch(const byte  ID, const unsigned short current);
  bool readPunch(const byte ID);

  // Low level public methods

  bool sendDxlCommand(const byte ID, const byte dxlCommand);
  bool sendDxlWrite(const byte ID, const byte dxlAddress, const byte *params, const byte nByteToBeWritten);
  bool sendDxlRegData(const byte ID, const byte dxlAddress, const byte *params, const byte nByteToBeWritten);
  bool sendDxlRead(const byte ID, const byte dxlAddress, const byte nByteToBeRead);
  bool dxlDataReady();
  unsigned short readDxlError();                      // return the error or DXL_ERR_SUCCESS of the last operation
  unsigned short readDxlResult();                     // return the value read of the last operation 
  
  bool isBusy();
  void setDxlTimeout(const unsigned long timeout) { _timeout = timeout; }
  unsigned long readDxlTimeout() { return _timeout; }
  void setDxlReturnDelayTime(const unsigned long returnDelayTime) { _returnDelayTime = returnDelayTime; }
  unsigned long readDxlReturnDelayTime() { return _returnDelayTime; }  
  
};

#endif // Dynamixel_h
