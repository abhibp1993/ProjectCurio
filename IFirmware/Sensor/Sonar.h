/************************************************************************************************************
 * Sonar.h - Provides wrapper classes, and functions for Sonar Interface                                    *
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


#ifndef SONAR_h
#define SONAR_h

  #include "Arduino.h"

  // Utility Class for easy maintainance of Digital MUX
  class Mux81{
    public:
      Mux81(uint8_t chA, uint8_t chB, uint8_t chC);
      void select(uint8_t pinNum);
  };
  
  // Provides a wrapper around ultrasonic sensor attached via digital multiplexer.
  class Sonar{
    
    private:
      Mux81* myMux;
      uint8_t echo, trig;
      
    public:
      float waitTimeInMillis;
      
      Sonar(uint8_t pinEcho, uint8_t pinTrig);
      Sonar(uint8_t pinEcho, uint8_t pinTrig, Mux81 muxTrig);
      
      float getDistance();  // returns distance
      float getRawTime();   // returns raw pulse duration
  };
  
  
  
  extern Mux81 digiMux;
  extern Mux81 analMux;
  
  extern Sonar  sonar1;
  extern Sonar  sonar2;
  extern Sonar  sonar3;
  extern Sonar  sonar4;
  extern Sonar  sonar5;
  
#endif
