/************************************************************************************************************
 * Conf.h - User configuration for adjusting firmware.                                                      *
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


/* User configuration File */


// Remark 1: Channel change should be avoided. ROS(=serial) is primary. 



#ifndef CONF_h
#define CONF_h
  
  #define CHANNEL_I2C         0      // communication channel = I2C
  #define CHANNEL_SER         1      // communication channel = Serial
  
  #define SELECT_CHANNEL      CHANNEL_SER  // default channel is serial (change if required)
  #define WHOAMI              99           // Address of Arduino for I2C and identity to be used for Serial communication 
                                           // (helpful in uniquely identifying this device)

//  #define IS_CONFIGURABLE      true      // if online configuration changes are permitted   (******* NOT CURRENTLY IMPLEMENTED *******)
  #define RAW_OR_PROCESSED     true        // if return data is raw encoder reading or processed motor speed (true -> processed)
  #define IS_VFDRIVE           true        // if variable frequency method is enabled (true: better performance)
  #define IS_PID_ACTIVE        false       // if internal PID is enabled. (whether or not to engage can be configured online)
  #define IS_ERROR_ON          false       // if error state reporting is ON/OFF
//  #define IS_HYBRID_SWITCH_ON  false     // if hybrid control switch enabled    (******* NOT CURRENTLY IMPLEMENTED *******)
  
  
  #define ENCODER_CPR          48*34.1   // encoder CPR rating (2 channel, both rising and falling edges should be counted)
  #define MOTOR_UBOUND         0.9       // Upper bound on motor PWM duty cycle
  #define MOTOR_LBOUND         0.08      // Lower bound on motor PWM duty cycle
  
  
  #define IS_CONST_PID         false     // if change of PID gains (online) is permitted
  #if (IS_CONST_PID == true)             // Set P, I, D constant gains in this section if dynamic setting is disabled.
    #define CONST_M1_Kp 1.0                 
    #define CONST_M1_Ki 0.0
    #define CONST_M1_Kd 0.0
    
    #define CONST_M2_Kp 1.0                 
    #define CONST_M2_Ki 0.0
    #define CONST_M2_Kd 0.0
  #endif
  
  
  #define IS_CURR_FEEDBACK    true      // if current feedback is disabled/enabled
  #if (IS_CURR_FEEDBACK == true)         // if current feedback is taken, set the upper bound for error generation/protection
    #define CURR_ERR   6.5               // maximum permissible current in Amps (set to value > 35 for bypassing the protection)
    #define CURR_WARN  5.5               // maximum permissible current in Amps (set to value > 35 for bypassing the protection)
    #define CURR_ITER   10               // size of SMA filter for motor current (don't keep it large, slows the measurement process)
  #endif
  
  #define IS_VOLT_FEEDBACK    false      // if voltage feedback is used
  #if (IS_VOLT_FEEDBACK == true)         // if current feedback is taken, set the upper bound for error generation/protection
    #define VOLT_ERR   7                 // maximum permissible current in voltage (set to value > 12 for bypassing the protection)
    #define VOLT_WARN  6                 // maximum permissible current in voltage (set to value > 12 for bypassing the protection)
    #define VOLT_ITER   10               // size of SMA filter for motor voltage (don't keep it large, slows the measurement process)
  #endif
  
  #define BATT_ITER   10                 // size of SMA filter for battery voltage (don't keep it large, slows the measurement process)
  
  #define ADC_RESOLUTION   0.0048828     // = 5.0/1024.0
  #define GAIN             4             // motor Current sense amplifier gain 
  #define M_CS_RESO  (7.69231 / GAIN)    // 1 / (130m * GAIN)
  
  
  
  //==================================================================
  // Error Codes: the error string can be changed according to firmware updates
  
  #define E001 "e001"      // Error: high current motor 1
  #define E002 "e002"      // Error: high current motor 2
  #define E003 "e003"      // Error: high voltage motor 1
  #define E003 "e004"      // Error: high voltage motor 2
  
  #define W001 "w001"      // Warning: high current motor 1
  #define W002 "w002"      // Warning: high current motor 2
  #define W003 "w003"      // Warning: high voltage motor 1
  #define W003 "w004"      // Warning: high voltage motor 2


  //==================================================================
  // Common variables
  
  
  
  
  
#endif
