/************************************************************************************************************
 * Motor.h - Handles motor, encoder interfacing with PID Controller                                         *
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


#ifndef MOTOR_h
#define MOTOR_h

  #include "Arduino.h"
  #include <avr/io.h>
  #include "Conf.h"
  
  #define MOTOR_MAX_SPEED 300.0
  
  class Motor;
  class Encoder;
  class PID;
  
  
  class Motor{

    private:                                                    // Private Variables
      uint8_t pwm, in1, in2;                                    // Control Pins
      float refSpeed, duty;                                     // Speed Control Parameters
      float speed;                                              // speed: in rpm (float)
      boolean _isPID, _isEncoder;                               // Configuration Parameters
    
    public:                                                     // Public Variables and Methods
      Motor(uint8_t pwm, uint8_t in1, uint8_t in2);             // Instantiation method
      
      boolean start;                                            // Start/Stop the motor (true: start, false: stop)
      boolean brake;                                            // Engage brake to motor
      boolean direction;                                        // direction: true = clockwise, false = anticlockwise
      
      Encoder* myEnc;                                           // Encoder Associated with this motor
      PID* myPID;                                               // PID Regulator associated with this motor
      
      float getSpeed();                                         // Returns the actual speed (if encoders used), else returns estimated speed.
      boolean getDirection();                                   // Returns true, if motor is currently rotating clockwise. Value is sensed if encoders are used.
      
      void setPWM(float duty);                                  // Sets the duty cycle of PWM. vfdEnabled optimizes the range of operation for Polulu Motor.
      void setSpeed(float rpm);                                 // Sets the speed of motor. (closed loop if encoders and pid is enabled. Else open loop).

  };
  
  
  class PID{
    
    public:
      float Kp, Ki, Kd;                                         // P, I, D Gains. Range: 0-1
      float lastErr, ITerm;
      
      PID();                                                    // Constructor
      float update(float reference, float current);             // Runs one update cycle.
  };
  
  
  class Encoder{
    
    private:
      uint8_t pinA, pinB;                                       // Pins to which channels are connected
      long int counts;                                          // counts store
      uint8_t errors;                                           // error store
      
      uint8_t lastValA;                                         // logic levels on channel A
      uint8_t lastValB;                                         // logic levels on channel B
      
      void enableInterrupts();                                  // Enable the PCINT interrupt on selected pins.
    
    public:
      Encoder(uint8_t chA, uint8_t chB);                        // Instantiation (255 or -1 indicates not set or used)
      
      long int getCounts();                                     // returns the counts since last reset.
      long int getCountsAndReset();                             // returns the counts and resets the counter.
      uint8_t checkError();                                     // Returns the error counting (unacceptable levels on channels)  
  };
  
  
  extern Motor m1;
  extern Motor m2;
  
#endif
