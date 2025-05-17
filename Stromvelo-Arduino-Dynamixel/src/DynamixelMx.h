/*
DynamixelMx.h
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
 This library implement all the required operation to drive Dynamixel MX servo,
 
 Limitations:
  - This library is non blocking, i.e. when a write or a read command occured, it will NOT wait the answer of the servo
  - Baudrate tested up to 400 000bps on 16Mhz Arduino board (Mega 2560), with MAX485 component

 documentation for MX actuator is here:
 http://support.robotis.com/beta/en/product/dynamixel/mx_series/mx-64.htm
*/
#ifndef DynamixelMx_h
#define DynamixelMx_h

#include "DynamixelMxConfig.h"
#include "Dynamixel.h"
#include "Arduino.h"

/******************************************************************************
* Definitions
******************************************************************************/

class dxlMx : public dxl
{
public:
  
  // public methods
  dxlMx(halfDuplexSerial* port);
  ~dxlMx();

  //
  //  EEPROM commands
  //

  bool setMultiturnOffset(const byte ID, const short Offset);
  bool readMultiturnOffset(const byte ID);

  bool setResolutionDivider(const byte  ID, const byte Resolution);
  bool readResolutionDivider(const byte ID);

  //
  //  RAM commands
  //

  bool setDGain(const byte  ID, const byte gain);
  bool readDGain(const byte ID);

  bool setIGain(const byte  ID, const byte gain);
  bool readIGain(const byte ID);

  bool setPGain(const byte  ID, const byte gain);
  bool readPGain(const byte ID);

  bool setCurrent(const byte  ID, const unsigned short current);
  bool readCurrent(const byte ID);

  bool setTorqueControlEnable(const byte ID, const bool Enable);
  bool isTorqueControlEnable(const byte ID);

  bool setGoalTorque(const byte  ID, const unsigned short Torque);
  bool readGoalTorque(const byte ID);

  bool setGoalAcceleration(const byte  ID, const byte Acceleration);
  bool readGoalAcceleration(const byte ID);
  
};

#endif // DynamixelMx_h
