/*
DynamixelAx.h
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
 using only one wire.
 It required the softHalfDuplexSerial lib
 Limitations:
  - The pin should support change interrupts (see halfDuplexSerial lib)
  - Baudrate is limited (57 600bps ~ 115 200bps for 16Mhz Arduino board)
  - 5V Arduino Board (I didn't try with 3.3V board as I didn't have one)
  - This library is non blocking, i.e. when a write or a read command occured, it will NOT wait the answer of the servo

For more information, please visit :
https://github.com/akira215/DynamixelArduino
*/
#ifndef DynamixelAx_h
#define DynamixelAx_h

#include "Dynamixel.h"
#include "DynamixelAxConfig.h"
#include "Arduino.h"


/******************************************************************************
* Definitions
******************************************************************************/

class dxlAx : public dxl
{
public:
  // public methods
  dxlAx(halfDuplexSerial* port);
  ~dxlAx();
 
  bool setCWComplianceMargin(const byte  ID, const byte margin);
  bool readCWComplianceMargin(const byte ID);

  bool setCCWComplianceMargin(const byte  ID, const byte margin);
  bool readCCWComplianceMargin(const byte ID);

  bool setCWComplianceSlope(const byte  ID, const byte slope);
  bool readCWComplianceSlope(const byte ID);

  bool setCCWComplianceSlope(const byte  ID, const byte slope);
  bool readCCWComplianceSlope(const byte ID);

};

#endif // DynamixelAx_h
