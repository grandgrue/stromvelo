/*
DynamixelMxConfig.h 
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
 This file contains all definition to drive Dynamixel servos (actually MX servos).
 You should modify the value according to your servo specification.
 Please visit http://support.robotis.com/en/product/dynamixel/mx_series/mx-64.htm for MX actuators description
 and http://support.robotis.com/en/product/dynamixel/dxl_communication.htm to understand Dynamixel communication protocol
*/

#ifndef DynamixelMxConfig_h
#define DynamixelMxConfig_h

/******************************************************************************
* Servo registers (ADDRESS)
******************************************************************************/

#define DXL_ADD_MULTITURN_OFFSET      (0x14)
#define DXL_ADD_RESOLUTION_DIVIDER    (0x16)
#define DXL_ADD_D_GAIN		            (0x1A)
#define DXL_ADD_I_GAIN 		            (0x1B)
#define DXL_ADD_P_GAIN   	            (0x1C)
#define DXL_ADD_CURRENT               (0X44)
#define DXL_ADD_TORQUE_CONTROL_ENABLE (0X46)
#define DXL_ADD_GOAL_TORQUE           (0X47)
#define DXL_ADD_GOAL_ACCELERATION     (0X49)

/**************************************************************/

#endif // DynamixelMxConfig_h
