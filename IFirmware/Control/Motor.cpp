/************************************************************************************************************
 * Motor.cpp - Implementation of methods in Motor.h                                               .         *
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
 
 
 #include "Motor.h"
 #include "PinConfig.h"
 
 Motor m1(M1_PWM, M1_IN1, M1_IN2);
 Motor m2(M2_PWM, M2_IN1, M2_IN2);
  
  
//==========================================================================================================
// Motor Class Functions:
 
/**********************************************************************************
Function: Motor (constructor)
Parameters: 
  1. pinPWM: pin which would generate PWM for speed control of motor.
  2. pinIN1: Direction control pin 1
  3. pinIN2: Direction control pin 2

Description:
  Instantiates a Motor object with corresponding pins. 
  
Last Modified:
  1. abhibp1993: 27 Aug 2015, 2130
**********************************************************************************/
Motor::Motor(uint8_t pwm, uint8_t in1, uint8_t in2){
  
}


/**********************************************************************************
Function: getSpeed
Parameters: None
Returns: float 
Description:
  Returns the speed of motor.
  If encoder is enabled and working fine, then the speed is sensed value. However, 
  if the encoder is not used, the speed is predicted from PWM duty cycle and motor
  parameters (voltage and max. speed)
  
Last Modified:
  1. abhibp1993: 27 Aug 2015, 2130
**********************************************************************************/
float Motor::getSpeed(){
  
}


/**********************************************************************************
Function: getDirection
Parameters: None
Returns: boolean. True = Clockwise, False otherwise (including stopped motor)
Description:
  Returns the direction of motor.
  If encoder is enabled and working fine, then the direction is sensed value. 
  However, if the encoder is not used, the direction is predicted from states of
  in1, in2 pins.
  
Last Modified:
  1. abhibp1993: 27 Aug 2015, 2130
**********************************************************************************/
boolean Motor::getDirection(){
  
}


/**********************************************************************************
Function: setPWM
Parameters:
  1. duty: a float between 0.0 and 1.0 denoting the percentage duty cycle.
  2. vfdEnabled: a boolean to enable or disable the Variable Frequency operation
      of the PWM generation circuit. Default, true = enabled.
      
Returns: None
Description:
  Sets the PWM duty cycle for the channel. 
  Variable frequency provides a larger range by tweaking the inductive impedance of
  motor. The frequencies used here are tested and optimized for Polulu 25mm Diameter 
  Metal Gearmotor (6V/6A, 285RPM).
  
Last Modified:
  1. abhibp1993: 27 Aug 2015, 2130
**********************************************************************************/
void Motor::setPWM(float duty, boolean vfdEnabled = true){
  
}


/**********************************************************************************
Function: setSpeed
Parameters:
  1. rpm: a float between -MAX_RPM to +MAX_RPM.
      
Returns: None
Description:
  Sets the speed of motor.
  If encoder AND PID are enabled, a closed loop implementation is possible. Else, 
  an open-loop prediction from applied PWM, MAX RPM, Voltage input is made and duty
  cycle of PWM changed accordingly.
  
Last Modified:
  1. abhibp1993: 27 Aug 2015, 2130
**********************************************************************************/
void Motor::setSpeed(float rpm){
  
}


//==========================================================================================================
// PID Class Functions:

/**********************************************************************************
Function: PID (constructor)
Parameters: None

Description:
  Instantiates a PID object with default gains of Kp = 1.0, Ki = Kd = 0.0 
  
Last Modified:
  1. abhibp1993: 27 Aug 2015, 2130
**********************************************************************************/
PID::PID(){
  
}


/**********************************************************************************
Function: update
Parameters:
  1. reference: a float in range [0, 1]. Represents the desired value of state.
  2. current: a float in range [0, 1]. Represents the current value of state.
      
Returns: 
  1. A float in range [-1, 1]. Represents the "differential" change to be made to 
     control signal.
  
Description:
  Applied one PID update. 
  The update implements a generic scaled PID controller where all inputs and outputs
  are scaled in range [0, 1]. The output is a differential signal in range [-1, 1] 
  denoting the "change" to be made in control signal. 
  
Last Modified:
  1. abhibp1993: 27 Aug 2015, 2130
**********************************************************************************/
float PID::update(float reference, float current){
  
}



//==========================================================================================================
// Encoder Class Functions:

/**********************************************************************************
Function: Encoder (constructor)
Parameters: 
  1. chA: Arduino Pin number as integer connected to Channel A of Encoder. (mandatory)
  2. chB: Arduino Pin number as integer connected to Channel B of Encoder. (optional)
      [Default: -1 --> No channel available]

Description:
  Instantiates an encoder object with single or two channels of encoders. 
  
Last Modified:
  1. abhibp1993: 27 Aug 2015, 2130
**********************************************************************************/
Encoder::Encoder(uint8_t chA, uint8_t chB = -1){
  
}


/**********************************************************************************
Function: getCounts
Parameters: None
Returns: 
  1. No. of counts (32-bit integer)
  
Description:
  Returns the number of counts since when the encoder is enabled.
  
Last Modified:
  1. abhibp1993: 27 Aug 2015, 2130
**********************************************************************************/
uint32_t Encoder::getCounts(){
  
}


/**********************************************************************************
Function: getCountsAndReset
Parameters: None
Returns: 
  1. No. of counts (32-bit integer)
  
Description:
  Reads the number of counts and resets. The number of counts depends on when the 
  reset was applied for last time. 
  
Last Modified:
  1. abhibp1993: 27 Aug 2015, 2130
**********************************************************************************/
uint32_t Encoder::getCountsAndReset(){
  
}


/**********************************************************************************
Function: checkError
Parameters: None
Returns: 
  1. Number of error states (32-bit integer)
  
Description:
  Reads the number of error states. 
  
Last Modified:
  1. abhibp1993: 27 Aug 2015, 2130
**********************************************************************************/
uint8_t Encoder::checkError(){
  
}
