/*
DynamixelAxConfig.h 
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
 This file contains all definition to drive Dynamixel servos (actually AX servo and most MX functions will work).
 You should modify the value according to your servo specification. Take care that invalid address or command can lead to unpredictible results
 Please visit http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm for AX actuators description
 and http://support.robotis.com/en/product/dynamixel/dxl_communication.htm to understand Dynamixel communication protocol
*/

#ifndef DynamixelAxConfig_h
#define DynamixelAxConfig_h


/******************************************************************************
* Servo registers (ADDRESS)
******************************************************************************/

#define DXL_ADD_CW_COMPLIANCE_MARGIN  (0x1A)
#define DXL_ADD_CCW_COMPLIANCE_MARGIN (0x1B)
#define DXL_ADD_CW_COMPLIANCE_SLOPE   (0x1C)
#define DXL_ADD_CCW_COMPLIANCE_SLOPE  (0x1D)

/**************************************************************/

#endif // DynamixelAxConfig_h
