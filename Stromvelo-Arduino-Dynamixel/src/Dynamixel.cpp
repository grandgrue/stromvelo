/*
Dynamixel.cpp
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

 Limitations:
  - Baudrate is limited 57 600bps ~ 115 200bps for 16Mhz Arduino board with softHalfDuplexSerial, and ~ 400 000bps for 16Mhz Arduino board with hardHalfDuplexSerial
  - This library is non blocking, i.e. when a write or a read command occured, it will NOT wait the answer of the servo
*/

//
// Includes
//
#include "Dynamixel.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

//
// Constructor
//
dxl::dxl(halfDuplexSerial* port) :
 _port(port), _error(0), _nByteToBeRead(0), 
 _currentId(0), _timeout(1000), _currentTimeout(0),
 _returnDelayTime(2000), _currentReturnDelayTime(0), _dxlResult(0)
{
}

//
// Destructor
//
dxl::~dxl()
{
  _port->end();
}

void dxl::begin(const unsigned long speed)
{
  _port->begin(speed);
}

//
// Dxl Commands
//
bool dxl::ping(const byte  ID)
{ return (dxl::sendDxlCommand(ID, DXL_PING)); }

bool dxl::action(const byte  ID)
{ return (dxl::sendDxlCommand(ID, DXL_ACTION)); }

bool dxl::reset(const byte  ID)
{ return (dxl::sendDxlCommand(ID, DXL_RESET)); }

bool dxl::reboot(const byte  ID)
{ return (dxl::sendDxlCommand(ID, DXL_REBOOT)); }


//
//  EEPROM commands
//

bool dxl::readModelNumber(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_MODEL_NUMBER , 2); } 

bool dxl::readFirmware(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_FIRMWARE , 1); } 

bool dxl::setId(const byte  ID, const byte newID)
{ return sendDxlWrite(ID, DXL_ADD_ID, &newID, 1 );} 
bool dxl::readId(const byte ID) 
{ return sendDxlRead(ID, DXL_ADD_ID , 1); } 
 
bool dxl::setBaudRate(const byte  ID, const byte baudRate)
{  return sendDxlWrite(ID, DXL_ADD_BAUDRATE, &baudRate, 1 );}
bool dxl::readBaudRate(const byte ID) 
{ return sendDxlRead(ID, DXL_ADD_BAUDRATE , 1); } 

bool dxl::setReturnDelayTime(const byte  ID, const byte returnDelayTime)
{  return sendDxlWrite(ID, DXL_ADD_RETURN_DELAY_TIME , &returnDelayTime, 1 );}
bool dxl::readReturnDelayTime(const byte ID) 
{ return sendDxlRead(ID, DXL_ADD_RETURN_DELAY_TIME , 1); } 

bool dxl::setCWAngleLimit(const byte  ID, const unsigned short angle)
{  return sendDxlWrite(ID, DXL_ADD_CW_ANGLE_LIMIT ,(const byte*) &angle, 2 );}
bool dxl::setCCWAngleLimit(const byte  ID, const unsigned short angle)
{  return sendDxlWrite(ID, DXL_ADD_CCW_ANGLE_LIMIT , (const byte*) &angle, 2 );}

bool dxl::readCWAngleLimit(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_CW_ANGLE_LIMIT , 2); } 
bool dxl::readCCWAngleLimit(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_CCW_ANGLE_LIMIT , 2); } 

bool dxl::setMaxTemperature(const byte  ID, const byte maxTemperature)
{ return sendDxlWrite(ID, DXL_ADD_MAX_TEMPERATURE , &maxTemperature, 1 );}
bool dxl::readMaxTemperature(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_MAX_TEMPERATURE , 1); } 

bool dxl::setMinVoltage(const byte  ID, const byte minVoltage)
{ return sendDxlWrite(ID, DXL_ADD_MIN_VOLTAGE , &minVoltage, 1 );}
bool dxl::readMinVoltage(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_MIN_VOLTAGE , 1); } 

bool dxl::setMaxVoltage(const byte  ID, const byte maxVoltage)
{ return sendDxlWrite(ID, DXL_ADD_MAX_VOLTAGE , &maxVoltage, 1 );}
bool dxl::readMaxVoltage(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_MAX_VOLTAGE , 1); } 

bool dxl::setMaxTorque(const byte  ID, const unsigned short maxTorque)
{ return sendDxlWrite(ID, DXL_ADD_MAX_TORQUE ,(const byte*) &maxTorque, 2 );}
bool dxl::readMaxTorque(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_MAX_TORQUE , 2); } 

bool dxl::setStatusReturnLevel(const byte  ID, const byte Status)
{ return sendDxlWrite(ID, DXL_ADD_STATUS_RETURN_LEVEL , &Status, 1 );}
bool dxl::readStatusReturnLevel(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_STATUS_RETURN_LEVEL , 1); } 
  
bool dxl::setAlarmLed(const byte  ID, const byte Status)
{ return sendDxlWrite(ID, DXL_ADD_ALARM_LED , &Status, 1 );}
bool dxl::readAlarmLed(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_ALARM_LED , 1); } 

bool dxl::setAlarmShutdown(const byte  ID, const byte Status)
{ return sendDxlWrite(ID, DXL_ADD_ALARM_SHUTDOWN , &Status, 1 );}
bool dxl::readAlarmShutdown(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_ALARM_SHUTDOWN , 1); } 


//
//  RAM commands
//
bool dxl::setTorqueEnable(const byte  ID, const bool Status)
{ return sendDxlWrite(ID, DXL_ADD_TORQUE_ENABLE, (const byte*) &Status, 1 );} 
bool dxl::readTorqueEnable(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_TORQUE_ENABLE , 1); } 

bool dxl::setLedEnable(const byte  ID, const bool Status)
{ return sendDxlWrite(ID, DXL_ADD_LED, (const byte*) &Status, 1 );} 
bool dxl::readLedEnable(const byte  ID )
{ return sendDxlRead(ID, DXL_ADD_LED , 1); } 

bool dxl::setGoalPosition(const byte ID, const short Position)
{ return sendDxlWrite(ID, DXL_ADD_GOAL_POSITION, (const byte*) &Position, 2 );} 

bool dxl::setGoalPositionAtSpeed(const byte ID, const short Position, const short Speed)
{ 
  const short params[] = { Position, Speed };
  return sendDxlWrite(ID, DXL_ADD_GOAL_POSITION, (const byte*) params, 4 );
}   

bool dxl::setMovingSpeed(const byte ID, const short Speed)
{ return sendDxlWrite(ID, DXL_ADD_MOVING_SPEED, (const byte*) &Speed, 2 );}

bool dxl::setTorqueLimit(const byte ID, const short torque)
{ return sendDxlWrite(ID, DXL_ADD_TORQUE_LIMIT, (const byte*) &torque, 2 );}

bool dxl::readPresentPosition(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_PRESENT_POSITION , 2); } 

bool dxl::readPresentSpeed(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_PRESENT_SPEED , 2); } 

bool dxl::readPresentLoad(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_PRESENT_LOAD , 2); } 

bool  dxl::readVoltage(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_PRESENT_VOLTAGE , 1); } 

bool dxl::readTemperature(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_PRESENT_TEMPERATURE, 1); }  

bool dxl::isRegistered(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_REGISTERED , 1); }   

bool dxl::isMoving(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_MOVING , 1); } 

bool dxl::setEEPROMLock(const byte ID, const bool lock)
{ return sendDxlWrite(ID, DXL_ADD_LOCK, (const byte*) &lock, 1 );}  
bool dxl::isEEPROMLock(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_LOCK , 1); } 

bool dxl::setPunch(const byte  ID, const unsigned short current)
{ return sendDxlWrite(ID, DXL_ADD_PUNCH ,(const byte*) &current, 2 );}
bool dxl::readPunch(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_PUNCH , 2); } 


//
// Low Level Public Methods
//

bool dxl::sendDxlCommand(const byte ID, const byte dxlCommand)
{
  byte sentence[6];
  byte Checksum = (~(ID + 0x02 + dxlCommand )) & 0xFF;

  sentence[0] = sentence[1] = DXL_START;  // 2 start bytes
  sentence[2] = ID;                       // Servo ID
  sentence[3] = 0x02;                     // length
  sentence[4] = dxlCommand;               // dxl instruction
  sentence[5] = Checksum;                 // Checksum
 
  if(writeRaw(sentence, 6))
  {
    _nByteToBeRead = 6;
    _currentId = ID;
    return true;
  }

  _nByteToBeRead = 0;
  _currentId = 0;
  return false;

}


bool dxl::sendDxlRegData(const byte ID, const byte dxlAddress, const byte *params, const byte nByteToBeWritten )
{
  byte i;
  const byte txDataLength =  nByteToBeWritten + 3 ;    // length is "number of parameters N +2"
  byte sentence[7 + nByteToBeWritten];
  byte Checksum = 0;

  for (i = 0; i < nByteToBeWritten; i++)
    Checksum += params[i];
    
  Checksum = (~(ID + txDataLength  + DXL_REG_WRITE + dxlAddress + Checksum )) & 0xFF;

  sentence[0] = sentence[1] = DXL_START;  // 2 start bytes
  sentence[2] = ID;                       // Servo ID
  sentence[3] = txDataLength;             // length
  sentence[4] = DXL_REG_WRITE;           // write instruction
  sentence[5] = dxlAddress;               // adress to start
             
  for (i = 0; i < nByteToBeWritten; i++)  // write params ( first one is address)
    sentence[i+6] = params[i];
  
  sentence[i+6] = Checksum;

  if(writeRaw(sentence, txDataLength + 4))
  {
    _nByteToBeRead = 6;
    _currentId = ID;
    return true;
  }

  _nByteToBeRead = 0;
  _currentId = 0;
  return false;
  
}

bool dxl::sendDxlWrite(const byte ID, const byte dxlAddress, const byte *params, const byte nByteToBeWritten)
{
  byte i;
  const byte txDataLength =  nByteToBeWritten + 3 ;    // length is "number of parameters N +2"
  byte sentence[7 + nByteToBeWritten];
  byte Checksum = 0;

  for (i = 0; i < nByteToBeWritten; i++)
    Checksum += params[i];
    
  Checksum = (~(ID + txDataLength  + DXL_WRITE_DATA + dxlAddress + Checksum )) & 0xFF;

  sentence[0] = sentence[1] = DXL_START;  // 2 start bytes
  sentence[2] = ID;                       // Servo ID
  sentence[3] = txDataLength;             // length
  sentence[4] = DXL_WRITE_DATA;           // write instruction
  sentence[5] = dxlAddress;               // adress to start
             
  for (i = 0; i < nByteToBeWritten; i++)  // write params ( first one is address)
    sentence[i+6] = params[i];
  
  sentence[i+6] = Checksum;

  if(writeRaw(sentence, txDataLength + 4))
  {
    _nByteToBeRead = 6;
    _currentId = ID;
    return true;
  }

  _nByteToBeRead = 0;
  _currentId = 0;
  return false;

}

bool dxl::sendDxlRead(const byte ID, const byte dxlAddress, const byte nByteToBeRead)
{

  byte Checksum = (~(ID +  4  + DXL_READ_DATA + dxlAddress + nByteToBeRead)) & 0xFF;
  byte sentence[] ={ DXL_START, DXL_START, ID, 0x04, DXL_READ_DATA, dxlAddress, nByteToBeRead, Checksum};

  if(writeRaw(sentence, 8))
  {
    _nByteToBeRead = 6 + nByteToBeRead;
    _currentId = ID;
    return true;
  }

  _nByteToBeRead = 0;
  _currentId = 0;
  return false;

}

bool dxl::isBusy()
{
  if(micros() < _currentReturnDelayTime)
    return true;
  else
    return false;  // We are waiting some answer
}

bool dxl::dxlDataReady()
{
  if(_nByteToBeRead == 0)
    return false;
 
  if(_port->available()>=_nByteToBeRead)
  {
    if(_currentReturnDelayTime == 0)
      _currentReturnDelayTime = _returnDelayTime + micros();

    return true;
  }

  if(millis()>=_currentTimeout )
  {
    _error |= DXL_ERR_RX_TIMEOUT;
    return true;
  }

  return false;
}

unsigned short dxl::readDxlResult() 
{  
  if(_nByteToBeRead == 0)
    return _dxlResult;
  else
    return readDxlData();	
}

unsigned short dxl::readDxlError()             
{  
  if(_nByteToBeRead != 0)
    readDxlData();
  
  return _error;	
}



//
// Private Methods
//

// The only function to write on serial port
bool dxl::writeRaw( const byte *sentence, const byte nByteToBeWritten)
{
   while(_port->available() > 0) 
    _port->read();            // clear the  Rx buffer

  _error = DXL_ERR_SUCCESS;  // purge error
  _dxlResult = 0;            // purge result

  if(_port-> write(sentence, nByteToBeWritten) == nByteToBeWritten)
  {
    _currentTimeout = millis() + _timeout;
    _currentReturnDelayTime = 0;
    return true;
  }
  else
  {
    _currentTimeout = 0;
    return false;
  }
}

// The only function to read on serial port
unsigned short dxl::readDxlData()
{
  if(_port->available()<_nByteToBeRead)
  {
    while(_port->available() > 0) 
      _port->read();            // clear the  Rx buffer
    _nByteToBeRead = 0;
    _currentTimeout = 0;
    _currentId = 0;
    return _error; 
  }

  byte check = 0;             // used to compute checksum
  byte header[5];             // 2 start byte + ID + length + Error
  byte nDataBytes = _nByteToBeRead - 6;
  byte result[nDataBytes];    // where the data will be saved
  
  // Read the incoming bytes
  header[0] = _port->read();   // 2 Starts Bytes
  header[1] = _port->read();   
    
  header[2] = _port->read();   // Servo ID
  header[3] = _port->read();   // Length
  header[4] = _port->read();   // Error

  for (byte i = 0; i < nDataBytes; i++)
  {
    result[i] = _port->read();
    check +=  result[i];
  }

  byte Checksum  = _port->read();   // Checksum
  
  if (!( ( header[0] == DXL_START) & ( header[1]  == DXL_START) ))
    _error |=  DXL_ERR_RX_FAIL;     // No header found

  if (header[2]!=_currentId)
    _error |=DXL_ERR_ID;            // The response comes from another Servo !

  if (header[3]!= nDataBytes + 2)
  {
    nDataBytes = header[3] - 2;
    _error |=DXL_ERR_RX_LENGTH;             // Servo send an error we push it. To catch it error mask should be used
  }

  if (header[4]!=0)
    _error |=header[4];             // Servo send an error we push it. To catch it error mask should be used
  

  if (Checksum != ((~(header[2] + header[3]  + header[4] + check )) & 0xFF ))
    _error |=DXL_ERR_RX_CORRUPT;    // Checksum error

  if(nDataBytes == 0)
    _dxlResult = 0;
  else if(nDataBytes == 1)
    _dxlResult = (unsigned short) result[0];
  else if(nDataBytes == 2)
    _dxlResult = (result[1] << 8) | result[0];   // (msb << 8) | lsb
    
  _nByteToBeRead = 0;
  _currentTimeout = 0;
  _currentId = 0;

  return  _dxlResult ;               // Returns the error, if OK, DXL_ERR_SUCCESS
}

