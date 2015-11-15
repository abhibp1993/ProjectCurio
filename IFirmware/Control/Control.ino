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
#include <ros.h>
#include <curio_msgs/motor.h>

#include "Conf.h"
#include "Motor.h"
#include "PinConfig.h"
#if (SELECT_CHANNEL == CHANNEL_SER)         // if current feedback is taken, set the upper bound for error generation/protection
  #include "CommSerial.h"
#else
  include "CommI2C.h"
#endif

// Global Variables
float m1RefSpeed;
float m2RefSpeed;

//ROS node Handle
ros::NodeHandle nh; // Arduino node

// Encoder Instantiation
Encoder e1(M1_CHA, M1_CHB);
Encoder e2(M2_CHA, M2_CHB);

// Motor Instantiation
Motor m1(M1_PWM, M1_IN1, M1_IN2);
Motor m2(M2_PWM, M2_IN1, M2_IN2);

void speed_callback( const curio_msgs::motor& speed_msg){
  m1RefSpeed = speed_msg.motor1;
  m2RefSpeed = speed_msg.motor2;
}

ros::Subscriber<curio_msgs::motor> speed_sub("speed_feedback", speed_callback );

/*
  Case Structure: 
    Check Single Motor performance on direct PWM duty application.
    
    1. RAW_OR_PROCESSED              false  --> pwm setting directly
    2. IS_VFDRIVE                    true
    3. IS_PID_ACTIVE                 false
    4. IS_ERROR_ON                   false
    5. IS_CONST_PID                  false
    6. IS_CURR_FEEDBACK              false
    7. IS_VOLT_FEEDBACK              false
*/
void testCase1(){
  static long int internalTime = micros();
  internalTime = micros() - internalTime;
  
  if (internalTime > 200000){    // update only after 200ms
    m1RefSpeed += 0.05;
    m2RefSpeed += 0.05;
    
    if (m1RefSpeed > 1.0) 
      m1RefSpeed = 0.0;
    if (m2RefSpeed > 1.0) 
      m2RefSpeed = 0.0;
      
  }
}

/*
  Case Structure: 
    Check Single Motor performance on speedInRPM input.
    
    1. RAW_OR_PROCESSED              true  --> provide reference speed in rpm
    2. IS_VFDRIVE                    true
    3. IS_PID_ACTIVE                 false
    4. IS_ERROR_ON                   false
    5. IS_CONST_PID                  false
    6. IS_CURR_FEEDBACK              false
    7. IS_VOLT_FEEDBACK              false
*/
void testCase2(){
  static long int internalTime = micros();
  internalTime = micros() - internalTime;
  
  if (internalTime > 200000){  //update after 200ms
    m1RefSpeed += 50;
    m2RefSpeed += 50;
    
    if (m1RefSpeed > MOTOR_MAX_SPEED) 
      m1RefSpeed = 0.0;
    if (m2RefSpeed > MOTOR_MAX_SPEED) 
      m2RefSpeed = 0.0;

  }
}

/*
  Case Structure: 
    Check Single Motor performance on speedInRPM input.
    
    1. RAW_OR_PROCESSED              true  --> provide reference speed in rpm
    2. IS_VFDRIVE                    true
    3. IS_PID_ACTIVE                 true  --> PID controller active
    4. IS_ERROR_ON                   false
    5. IS_CONST_PID                  true  --> for testing, use constant PID gains
    6. IS_CURR_FEEDBACK              false
    7. IS_VOLT_FEEDBACK              false
*/
void testCase3(){
  static long int internalTime = micros();
  internalTime = micros() - internalTime;
  
  if (internalTime > 200000){  //update after 200ms
    m1RefSpeed += 50;
    m2RefSpeed += 25;
    
    if (m1RefSpeed > MOTOR_MAX_SPEED) 
      m1RefSpeed = 0.0;
    if (m2RefSpeed > MOTOR_MAX_SPEED) 
      m2RefSpeed = 0.0;
  }
}



void setup() {
  
  // DG pins pull-up
  // ---
  
  //Ros setup
  nh.initNode();
  nh.subscribe(speed_sub);
    
  m1.myEnc = &e1;
  m1._isEncoder = true;
  m2.myEnc = &e2;
  m2._isEncoder = true;
  
  // PID Instantiation
  #if (IS_PID_ACTIVE == true)
    PID pid1;
    PID pid2;
    m1.myPID = &pid1;
    m2.myPID = &pid2;    
    
    #if (IS_CONST_PID == true)
      m1.myPID.Kp = CONST_M1_Kp;
      m1.myPID.Ki = CONST_M1_Ki;
      m1.myPID.Kd = CONST_M1_Kd;
      
      m2.myPID.Kp = CONST_M2_Kp;
      m2.myPID.Ki = CONST_M2_Ki;
      m2.myPID.Kd = CONST_M2_Kd;
    #endif
  #endif
  
}


long int time;
void loop() {
  // Call (Uncomment) appropriate test case.
  //testCase1();
  //testCase2();
  //testCase3();
  
  nh.spinOnce();
  
  // Check Time Stamp
  time = micros();
  
  // Check Battery Voltage
  float battVoltage = 0;
  for (int i = 0; i < BATT_ITER; i++){
    battVoltage += analogRead(BATT_V);
  }
  battVoltage = battVoltage * ADC_RESOLUTION / BATT_ITER;
  
  
  // Check Current through motors
  #if (IS_CURR_FEEDBACK == true)
  
    float m1Current = 0, m2Current = 0;
    for (int i = 0; i < CURR_ITER; i++){
      m1Current += analogRead(M1_CS);
      m2Current += analogRead(M2_CS);
    }
    m1Current = m1Current * ADC_RESOLUTION / CURR_ITER * M_CS_RESO;
    m2Current = m2Current * ADC_RESOLUTION / CURR_ITER * M_CS_RESO ;
    
  #endif
  
  
  // Check Voltage across motors
  #if (IS_VOLT_FEEDBACK == true)
  
    float m1Voltage = 0, m2Voltage = 0;
    for (int i = 0; i < VOLT_ITER; i++){
      m1Voltage += analogRead(M1_CS);
      m2Voltage += analogRead(M2_CS);
    }
    m1Voltage = m1Voltage * ADC_RESOLUTION / VOLT_ITER ;
    m2Voltage = m2Voltage * ADC_RESOLUTION / VOLT_ITER ;
  
  #endif
  
  
  // Check Motor Diagnostic pins
  unsigned char m1Diag, m2Diag; 
  m1Diag = (digitalRead(M1_DG) == HIGH) ? HIGH : LOW;
  m2Diag = (digitalRead(M2_DG) == HIGH) ? HIGH : LOW;
  
  // Primary Protection 
  if (battVoltage < 7.0){  // Low battery voltage protection
    // Error -------------
  }
  
  #if (IS_CURR_FEEDBACK == true)
    
    if (m1Current > CURR_WARN && m1Current < CURR_ERR)  {
      // warn -------------
    }
    else if (m1Current > CURR_ERR) { 
      // Error ------------
    }
    
    if (m2Current > CURR_WARN && m2Current < CURR_ERR)  {
      // warn -------------
    }
    else if (m2Current > CURR_ERR) { 
      // Error ------------
    }
    
  #endif
  
  
  #if (IS_VOLT_FEEDBACK == true)
    
    if (m1Voltage > VOLT_WARN && m1Voltage < VOLT_ERR)  {
      // warn -------------
    }
    else if (m1Voltage > VOLT_ERR) { 
      // Error ------------
    }
    
    if (m2Voltage > VOLT_WARN && m2Voltage < VOLT_ERR)  {
      // warn -------------
    }
    else if (m2Voltage > VOLT_ERR) { 
      // Error ------------
    }
    
  #endif
  
  
  // Motor Functionality ===================================================
  // Get recent reference speed
  float m1CurrSpeed = m1.getSpeed();
  float m2CurrSpeed = m2.getSpeed();
  
  // Based on configuration, apply the update
  #if (RAW_OR_PROCESSED == true)
    m1.setSpeed(m1RefSpeed);
    m2.setSpeed(m2RefSpeed);
  #else
    m1.setPWM(m1RefSpeed);
    m2.setPWM(m2RefSpeed);
  #endif
  
  
  // Communication
  /* Serial.println("-----");
  Serial.print("Batt V: "); Serial.println(battVoltage);
  #if (IS_CURR_FEEDBACK == true)
    Serial.print("M1 I: "); Serial.println(m1Current);
    Serial.print("M2 I: "); Serial.println(m2Current);    
  #endif
  #if (IS_VOLT_FEEDBACK == true)
    Serial.print("M1 V: "); Serial.println(m1Voltage);
    Serial.print("M2 V: "); Serial.println(m2Voltage);
  #endif
  Serial.print("M1 refS: "); Serial.print(m1RefSpeed); Serial.print("\tM1 currS: "); Serial.println(m1CurrSpeed);
  Serial.print("M2 refS: "); Serial.print(m2RefSpeed); Serial.print("\tM2 currS: "); Serial.println(m2CurrSpeed);
  
  time = micros() - time;
  Serial.print("Time: "); Serial.println(time);
  
  if (Serial.available() > 1){
    m1RefSpeed = Serial.parseFloat();
    m2RefSpeed = Serial.parseFloat();
    Serial.println("ACK");
    delay(20);
  } */
  
}
