/************************************************************************************************************
 * Control.ino - Firmware base for Control Arduino                                                .         *
 * This file is a part of ProjectCurio.                                                                     *
 *												            *															*
 * Copyright 2015 Abhishek N. Kulkarni (abhi.bp1993@gmail.com)		                                    *
 * The latest firmware is available at https://github.com/abhibp1993/ProjectCurio/              	    *
 * Last update: 26 Aug 2015                                                                                 *
 ************************************************************************************************************
 
 ************************************************************************************************************
 * This program is free software: you can redistribute it and/or modify					    *
 * it under the terms of the GNU General Public License as published by					    *
 * the Free Software Foundation, either version 3 of the License, or					    *
 * (at your option) any later version.									    *
 *													    *
 * This program is distributed in the hope that it will be useful,					    *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of					    *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the					    *
 * GNU General Public License for more details.								    *
 *													    *
 * You should have received a copy of the GNU General Public License					    *
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.			            *
 ***********************************************************************************************************/


#include "Conf.h"
#include "Motor.h"
#include "PinConfig.h"
#if (SELECT_CHANNEL == CHANNEL_SER)         // if current feedback is taken, set the upper bound for error generation/protection
  #include "CommSerial.h"
#else
  include "CommI2C.h"
#endif



void setup() {
  
}

void loop() {
  
  // Check Battery Voltage
  
  // Check Current through motors
  
  // Check Voltage across motors
  
  // 

}
