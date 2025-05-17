/*
DynamixelMx.cpp
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
 This library implement all the required operation to drive Dynamixel MX servo,
 
 Limitations:
  - This library is blocking, i.e. when a write or a read command occured, it will wait the answer of the servo
  - Baudrate tested up to 400 000bps on 16Mhz Arduino board (Mega 2560), with MAX485 component

 documentation for MX actuator is here:
 http://support.robotis.com/beta/en/product/dynamixel/mx_series/mx-64.htm
*/


//
// Includes
//
#include "DynamixelMx.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

//
// Constructor
//
dxlMx::dxlMx(halfDuplexSerial* port) :
 dxl(port)
{
}

//
// Destructor
//
dxlMx::~dxlMx()
{}

//
//  EEPROM commands
//

bool dxlMx::setMultiturnOffset(const byte ID, const short Offset)
{ return sendDxlWrite(ID, DXL_ADD_MULTITURN_OFFSET, (const byte*) &Offset, 2 );}   
bool dxlMx::readMultiturnOffset(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_MULTITURN_OFFSET , 2); } 

bool dxlMx::setResolutionDivider(const byte  ID, const byte Resolution)
{ return sendDxlWrite(ID, DXL_ADD_RESOLUTION_DIVIDER , &Resolution, 1 );}
bool dxlMx::readResolutionDivider(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_RESOLUTION_DIVIDER , 1); } 

//
//  RAM commands
//

bool dxlMx::setDGain(const byte  ID, const byte gain)
{ return sendDxlWrite(ID, DXL_ADD_D_GAIN , &gain, 1 );}
bool dxlMx::readDGain(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_D_GAIN , 1); } 

bool dxlMx::setIGain(const byte  ID, const byte gain)
{ return sendDxlWrite(ID, DXL_ADD_I_GAIN , &gain, 1 );}
bool dxlMx::readIGain(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_I_GAIN , 1); } 

bool dxlMx::setPGain(const byte  ID, const byte gain)
{ return sendDxlWrite(ID, DXL_ADD_P_GAIN , &gain, 1 );}
bool dxlMx::readPGain(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_P_GAIN , 1); } 

bool dxlMx::setCurrent(const byte  ID, const unsigned short current)
{ return sendDxlWrite(ID, DXL_ADD_CURRENT ,(const byte*) &current, 2 );}
bool dxlMx::readCurrent(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_CURRENT , 2); } 

bool dxlMx::setTorqueControlEnable(const byte ID, const bool Enable)
{ return sendDxlWrite(ID, DXL_ADD_TORQUE_CONTROL_ENABLE, (const byte*) &Enable, 1 );} 
bool dxlMx::isTorqueControlEnable(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_TORQUE_CONTROL_ENABLE  , 1); } 

bool dxlMx::setGoalTorque(const byte  ID, const unsigned short Torque)
{ return sendDxlWrite(ID, DXL_ADD_GOAL_TORQUE ,(const byte*) &Torque, 2 );}
bool dxlMx::readGoalTorque(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_GOAL_TORQUE , 2); } 

bool dxlMx::setGoalAcceleration(const byte  ID, const byte Acceleration)
{ return sendDxlWrite(ID, DXL_ADD_GOAL_ACCELERATION , &Acceleration, 1 );}
bool dxlMx::readGoalAcceleration(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_GOAL_ACCELERATION , 1); } 


