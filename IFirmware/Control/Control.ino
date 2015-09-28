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
//ROS imports
#include <ros.h>
#include <curio_msgs/motor.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <curio_msgs/gain.h>

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

//ROS message object
curio_msgs::motor motor_speed;
curio_msgs::motor motor_current;
curio_msgs::motor motor_voltage;
curio_msgs::motor motor_distance;
std_msgs::String error_string;
std_msgs::Float32 battery_voltage;

// Motor Instantiation
Motor m1(M1_PWM, M1_IN1, M1_IN2);
Motor m2(M2_PWM, M2_IN1, M2_IN2);

//ROS publishers
ros::Publisher speed_pub("curio_speed", &motor_speed);
ros::Publisher current_pub("curio_current", &motor_current);
ros::Publisher voltage_pub("curio_voltage", &motor_voltage);
ros::Publisher distance_pub("curio_distance", &motor_distance);
ros::Publisher error_pub("curio_control_error", &error_string);
ros::Publisher batteryVol_pub("curio_battery_voltage", &battery_voltage);

//Subsciber function
void gain_callback(const curio_msgs::gain& gain_msg){
      //do nothing
  
}
//ROS Subscribers (Always make subscriber below the subscription function)
ros::Subscriber<curio_msgs::gain> gain_sub("curio_gain", gain_callback );

//test
char hello[13] = "Error";
long int timeSetup = 0;
long int timeLoop = 0;

void setup() {
  
  //ROS Node setup
  nh.initNode();
  
  //Advertise publishers
  nh.advertise(speed_pub);
  nh.advertise(voltage_pub);
  nh.advertise(current_pub);
  nh.advertise(batteryVol_pub);
  nh.advertise(distance_pub);
  nh.advertise(error_pub);
  
  //Subscriber
  nh.subscribe(gain_sub);
 
  // DG pins pull-up
  // ---
  
  
  // Encoder Instantiation
  Encoder e1(M1_CHA, M1_CHB);
  Encoder e2(M2_CHA, M2_CHB);
  m1.myEnc = &e1;
  m2.myEnc = &e2;
  
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
  
  timeSetup = micros() - timeSetup;
}

void loop() {
  
  timeLoop = micros() - timeLoop;
  
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
  
  
  // Communication Packet Ready
    // CT: Always --> Motor Speeds
    // CT: Conf --> Motor Current
    // CT: Conf --> Motor Voltage
    // CT: Conf --> Battery Voltage
    // Errors
    
    // Call an rosUpdate function for transmission.
  motor_speed.motor1 = m1CurrSpeed;
  motor_speed.motor2 = m2CurrSpeed;
  
  #if (IS_CURR_FEEDBACK == true)
    motor_current.motor1 = m1Current;
    motor_current.motor2 = m2Current;
  #endif
  
  #if (IS_VOLT_FEEDBACK == true)
    motor_voltage.motor1 = m1Voltage;
    motor_voltage.motor2 = m2Voltage;
  #endif
  
  battery_voltage.data = battVoltage;
  
  error_string.data = hello;
  
  motor_speed.motor1 = timeSetup;
  motor_speed.motor2 = timeLoop;
  
  error_pub.publish( &error_string );
  speed_pub.publish( &motor_speed );
  
  nh.spinOnce();

}
