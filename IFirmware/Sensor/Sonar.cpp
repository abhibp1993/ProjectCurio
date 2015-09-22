/************************************************************************************************************
 * Sonar.cpp - Implements functionality in Sonar.h                                                          *
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


#include "Sonar.h"
#include "Conf.h"
#include "PinConfig.h"

Mux81 digiMux(DMUX_A, DMUX_B, DMUX_C);
Mux81 analMux(AMUX_A, AMUX_B, AMUX_C);

Sonar  sonar1(SONAR1_TRIG, SONAR1_ECHO, digiMux);
Sonar  sonar2(SONAR2_TRIG, SONAR2_ECHO, digiMux);
Sonar  sonar3(SONAR3_TRIG, SONAR3_ECHO, digiMux);
Sonar  sonar4(SONAR4_TRIG, SONAR4_ECHO, digiMux);
Sonar  sonar5(SONAR5_TRIG, SONAR5_ECHO, digiMux);


/**********************************************************************************
Function: Sonar (constructor - overload 1)
Parameters: 
  1. pinEcho: Arduino pin connected to Echo pin of ultrasonic sensor. 
  2. pinTrig: Arduino pin connected to Trigger pin of ultrasonic sensor.

Description:
  Instantiates a Sonar object with corresponding pins. 
  Default "waitTimeInMillis" is set to 10ms.
  
Last Modified:
  1. abhibp1993: 08 Sept 2015, 1810
  2. shrutiphadke: 10 Sept 2015 ,1415
**********************************************************************************/
Sonar::Sonar(uint8_t pinEcho, uint8_t pinTrig){

echo = pinEcho;
trig =pinTrig;
waitTimeInMillis=10.0;
}


/**********************************************************************************
Function: Sonar (constructor - overload 2)
Parameters: 
  1. pinEcho: Arduino pin connected to Echo pin of ultrasonic sensor. 
  2. pinTrig: Digital Mux pin connected to Trigger pin of ultrasonic sensor.
  3. muxTrig: Mux81 instance interfacing a Digital Multiplexer.

Description:
  Instantiates a Sonar object with corresponding pins. 
  Default "waitTimeInMillis" is set to 10ms.
  
Last Modified:
  1. abhibp1993: 08 Sept 2015, 1810
  2. shrutiphadke: 10 Sept 2015 ,1415
**********************************************************************************/
Sonar::Sonar(uint8_t pinEcho, uint8_t pinTrig, Mux81 muxTrig){
  echo =pinEcho;
  IS_MUX_USED =true;
  trig =pinTrig;
  myMux= &muxTrig;
  
}


/**********************************************************************************
Function: getDistance
Parameters: None
Returns: 
  Float: Distance of obstacle sensed (in cm).

Description:
  Returns the distance to obstacle in centimeters. Negative value implies no 
  obstacle.
  
Warnings:
  "w???": no obstacle ahead. 
  
Remarks:
  Use getRawTime method to get the time rather than repeating the same code in both
  functions.
  
Last Modified:
  1. abhibp1993: 08 Sept 2015, 1810
  2. shrutiphadke: 10 Sept 2015 ,1415
**********************************************************************************/
float Sonar::getDistance(){
  float tm = getRawTime();
  
  float dist;
  
  dist = (tm*0.314/2)*10;
  return dist;
  
}


/**********************************************************************************
Function: getRawTime
Parameters: None
Returns: 
  Float: Pulse duration on echo pin of ultrasonic sensor.

Description:
  Returns the distance to obstacle in centimeters. Negative value implies no 
  obstacle.
  
Warnings:
  "w???": no obstacle ahead. 
  
Last Modified:
  1. abhibp1993: 08 Sept 2015, 1810
  2. shrutiphadke: 10 Sept 2015 ,1415
**********************************************************************************/
float Sonar::getRawTime(){
  float duration;
  duration = pulseIn(echo,HIGH);
  duration =duration/1000;
  return duration;
}




Mux81::Mux81(uint8_t chA, uint8_t chB, uint8_t chC){
  
this->chA =chA;
this->chB =chB;
this->chC= chC;
}


/**********************************************************************************
Function: select
Parameters: None
Returns: None

Description:
  Selects the corresponding pin on Multiplexer.
  (Caution: Numbering is NOT 0-based, but starts from 1)

Remarks: chC is the LSB
  
Last Modified:
  1. abhibp1993: 08 Sept 2015, 1810
  2. shrutiphadke: 10 Sept 2015 ,1415
**********************************************************************************/
void Mux81::select(uint8_t pinNum){
 switch(pinNum)
 {
 
 case 1:  digitalWrite(chA, LOW);
          digitalWrite(chB, LOW);
          digitalWrite(chC, LOW);
          break;
          
 case 2:  digitalWrite(chA, LOW);
          digitalWrite(chB, LOW);
          digitalWrite(chC, HIGH);
          break;
          
 case 3:  digitalWrite(chA, LOW);
          digitalWrite(chB, HIGH);
          digitalWrite(chC, LOW);
          break;
          
 case 4:  digitalWrite(chA, LOW);
          digitalWrite(chB, HIGH);
          digitalWrite(chC, HIGH);
          break;
          
 case 5:  digitalWrite(chA, HIGH);
          digitalWrite(chB, LOW);
          digitalWrite(chC, LOW);
          break;
 
 }
}
 
