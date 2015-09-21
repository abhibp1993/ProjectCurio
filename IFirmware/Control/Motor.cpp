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
 
 //Motor m1(M1_PWM, M1_IN1, M1_IN2);
 //Motor m2(M2_PWM, M2_IN1, M2_IN2);
  
  
  
//==========================================================================================================
// Helper Functions 

/* [HELPER]**********************************************************************
Structure: IOStruct 

Description:
  Stores the AVR based Register pointers for the given pin
  
Refer PoluluMotor Library by Abhishek N. Kulkarni (abhibp1993)
Adopted from macegr's code. (Post: http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1235060559/0#4)
**********************************************************************************/
typedef struct IOStruct
{
	// if these aren't volatile, the compiler sometimes incorrectly optimizes away operations involving these registers:
	volatile unsigned char* pinRegister;
	volatile unsigned char* portRegister;
	volatile unsigned char* ddrRegister;
	unsigned char bitmask;
} IOPin;


/* [HELPER]**********************************************************************
Function: getIORegisters 
Parameters: 
  1. pin: pin for which the registers need to be found out. 
  2. io: IOPin instance. 

Description:
  The function modifies the IOPin structure to contain the respective registers. 

  $$ Only digital pins are allowed. $$

Refer to Polulu's OrangutanDigital Library.
**********************************************************************************/
void getIORegisters(unsigned char pin, IOPin* io){
  io->pinRegister = 0;
  io->portRegister = 0;
  io->ddrRegister = 0;
  io->bitmask = 0;
  
  if (pin < 8){			// pin 0 = PD0, ..., 7 = PD7
    io->pinRegister = (unsigned char*)&PIND;
    io->portRegister = (unsigned char*)&PORTD;
    io->ddrRegister = (unsigned char*)&DDRD;
    io->bitmask = 1 << pin;
  }
  else if (pin < 14){		// pin 8 = PB0, ..., 13 = PB5 (PB6 and PB7 reserved for external clock)
    io->pinRegister = (unsigned char*)&PINB;
    io->portRegister = (unsigned char*)&PORTB;
    io->ddrRegister = (unsigned char*)&DDRB;
    io->bitmask = 1 << (pin - 8);
  }
  else{
    // warning --------
  }
}

/* [HELPER]**********************************************************************
Function: setPWMFrequency 
Parameters: 
  1. pin: PWM pin on which the frequency is to be set. 
  2. divisor: prescaling factor to set the frequency.

Description:
  The function changes the frequency on pwm pin. 

  $$ Currently only pin 9, 10 frequency can be changed. $$
  $$ Changing frequency on either pin changes the pwm-frequency on other also $$

Refer PoluluMotor Library by Abhishek N. Kulkarni (abhibp1993)
Adopted from macegr's code. (Post: http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1235060559/0#4)
**********************************************************************************/
void setPWMFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    //TCCR2B = TCCR2B & 0b11111000 | mode;  // bug corrected from PoluluMotor class.
    TCCR1B = TCCR1B & 0b11111000 | mode;
  }
}
 
 
/* [HELPER]**********************************************************************
Function: getDivisor 
Parameters: 
  1. pin: pin for which the duty is being provided.
  1. duty: desired duty cycle on the pwm pin. [Value in range 0-1]

Description:
  The function returns an appropriate PWM-frequency so as to deliver enough current
  on pwm pin to drive the motor. (a good choice in case of high torque motors)
  
  $$ The slabs are chosen according to Polulu 25D metal gearmotor 6V/6A $$
  
Adopted and modified from PoluluMotor Library by Abhishek N. Kulkarni (abhibp1993)
**********************************************************************************/
int getDivisor(uint8_t pin, double duty){
  
  static float pin9Duty, pin10Duty;
  
  if       (pin == 9) { pin9Duty  = duty; }
  else if  (pin == 10){ pin10Duty = duty; }
  
  duty = min(pin9Duty, pin10Duty);
  
  if       (duty < 0.08) {}
  else if  (duty >= 0.08  && duty < 0.30)   { return 1024;}
  else if  (duty >= 0.30  && duty < 0.50)   { return 256 ;}  
  else if  (duty >= 0.50  && duty < 1.00)   { return 64  ;}
  else if  (duty >= 1.00)                   { return 64  ;}
}

  
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
  2. abhibp1993: 11 Sep 2015, 1000
**********************************************************************************/
Motor::Motor(uint8_t pwm, uint8_t in1, uint8_t in2){
  this->pwm = pwm;
  this->in1 = in1;
  this->in2 = in2;
  
  this->refSpeed = 0;
  this->duty = 0;
  this->direction = true;
}


/**********************************************************************************
Function: getSpeed
Parameters: None
Returns: float 
Description:
  Returns the speed of motor.
  If encoder is enabled and working fine, then the speed is sensed value. However, 
  if the encoder is not used, the speed is predicted from PWM duty cycle and motor
  parameters (suty and max. speed)
  
Last Modified:
  1. abhibp1993: 27 Aug 2015, 2130
  2. abhibp1993: 11 Sep 2015, 1000
**********************************************************************************/
float Motor::getSpeed(){
  static long int _lastRunTime;
  long int _now = millis() - _lastRunTime;
  
  if (this->_isEncoder){
    long int count = this->myEnc->getCountsAndReset();
    this->speed = (count/1636.8) * (1000 / _now) * 60; 
  }
  else{
    this->speed = MOTOR_MAX_SPEED * this->duty;
  }
  
  return this->speed;
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
  2. abhibp1993: 19 Sep 2015, 1049 (ignored encoder based direction sense.)
**********************************************************************************/
boolean Motor::getDirection(){
  return direction;
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

Remarks:
  If duty > maximum permitted bound, the duty is set to maximum value (clamped)
  If duty < minimum permitted bound, the motor is stopped.
  
Last Modified:
  1. abhibp1993: 27 Aug 2015, 2130
  2. abhibp1993: 11 Sep 2015, 1043 (scrap: reason -- unorganized)
  3. abhibp1993: 19 Sep 2015, 1041 (made changes in structure of motor to make life
                                    easy: -ve duty => reverse run. No rev flag. )
**********************************************************************************/
void Motor::setPWM(float duty){

  // negative duty
  if (duty < 0){
    direction = false;
    duty = -duty;
  }
  else{
    direction = true;
  }
  
  
  // Clip duty
  if (duty > MOTOR_UBOUND){
    start = true;
    duty = MOTOR_UBOUND;
    // warning ------
  }
  else if (duty < MOTOR_LBOUND){ 
    start = true; 
    brake = true;
    // warning ------
  }
  else{
    start = false;
  }
  
  
  // If motor is "stop", then disconnect
  if (start = false){
    pinMode(pwm, INPUT);
  }
  else{ // motor is "start", so start it!

    // check braking
    if (brake = true){
      digitalWrite(in1, HIGH);
      digitalWrite(in2, HIGH);
      delayMicroseconds(500);      // What should be this delay? Ideally? Theoretically -> may be based on motor model. 
    }
    
    digitalWrite(in1, direction);
    digitalWrite(in2, !direction);
    pinMode(pwm, OUTPUT);
    
    analogWrite(pwm, (int)(duty * 255));
  }
  
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
  
  if (!this->_isEncoder && this->_isPID){
    // to do
  }
  else{
    this->setPWM(float(rpm) / MOTOR_MAX_SPEED);
  }
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
  2. abhibp1993: 19 Sep 2015, 1505
**********************************************************************************/
PID::PID(){
  Kp = 1.0;
  Ki = 0.0;
  Kd = 0.0;
  
  lastErr = 0.0;
  ITerm = 0.0;  
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

Remark: 
  The inputs outside the (-1, 1) range are clipped off. 
  
Last Modified:
  1. abhibp1993: 27 Aug 2015, 2130
  2. abhibp1993: 19 Sep 2015, 1505
**********************************************************************************/
float PID::update(float reference, float current){
  
  // check reference and current values
  if (reference < -1 || reference > 1){
    reference = constrain(reference, -1, 1);
    // warning ----------
  }

  if (current < -1 || current > 1){
    current = constrain(current, -1, 1);
    // warning ----------
  }
  
  
  // Compute the error
  float err = reference - current;
  ITerm += err;
  float diffErr = err - lastErr;
  
  // Compute correction
  float changeInOutput = Kp*err + Ki*ITerm + Kd*diffErr;
  changeInOutput = constrain(changeInOutput, -1, 1);
  
  // Book-keeping and return
  lastErr = err;
  return changeInOutput;
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

Remark:
  For now (19 Sep 2015) we assume both encoder channels MUST be used.
  Hence, no handle for chB = -1 is coded.
  
Last Modified:
  1. abhibp1993: 27 Aug 2015, 2130
  2. abhibp1993: 19 Sep 2015, 1530
**********************************************************************************/
Encoder::Encoder(uint8_t chA, uint8_t chB = -1){
  
  cli();
  
  counts = 0;
  pinA = chA;
  pinB = chB;
  
  enableInterrupts();
  lastValA = digitalRead(pinA);
  lastValB = digitalRead(pinB);
  
  PCIFR = 0xFF;  // Clear all interrupt flags
  sei();
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
  2. abhibp1993: 19 Sep 2015, 1530
**********************************************************************************/
long int Encoder::getCounts(){
  cli();
  long int temp = counts;
  sei();
  return temp;
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
  2. abhibp1993: 19 Sep 2015, 1530
**********************************************************************************/
long int Encoder::getCountsAndReset(){
  cli();
  long int temp = counts;
  counts = 0;
  sei();
  return temp;
}


/**********************************************************************************
Function: checkError
Parameters: None
Returns: 
  1. Number of error states (8-bit integer)
  
Description:
  Reads the number of error states. 
  
Last Modified:
  1. abhibp1993: 27 Aug 2015, 2130
  2. abhibp1993: 19 Sep 2015, 1530
**********************************************************************************/
uint8_t Encoder::checkError(){
  cli();
  uint8_t temp = errors;
  sei();
  return temp;
}

void Encoder::enableInterrupts(){
  IOPin ioA, ioB;
  getIORegisters(pinA, &ioA);
  getIORegisters(pinB, &ioB);
  
  // chA, chB enable Interrupt Mask
  if (ioA.pinRegister == &PINB)
    PCMSK0 |= ioA.bitmask;
  if (ioA.pinRegister == &PINC)
    PCMSK1 |= ioA.bitmask;
  if (ioA.pinRegister == &PIND)
    PCMSK2 |= ioA.bitmask;
    
  if (ioB.pinRegister == &PINB)
    PCMSK0 |= ioB.bitmask;
  if (ioB.pinRegister == &PINC)
    PCMSK1 |= ioB.bitmask;
  if (ioB.pinRegister == &PIND)
    PCMSK2 |= ioB.bitmask;
 
  // Set data direction register 
  *ioA.ddrRegister &= ~ioA.bitmask;
  *ioB.ddrRegister &= ~ioB.bitmask;
  
  // Enable PCINT interrupt
  PCICR = 0xFF;
}


// ISR Handle --> Cannot access the internal variables of motors. 
// How to resolve this issue --> PLEASE THINK!!
ISR(PCINT0_vect)
{
//  uint8_t valA = digitalRead(pinA);
//  uint8_t valB = digitalRead(pinB);
//  
//  uint8_t plus  = valA ^ lastValB;
//  uint8_t minus = valB ^ lastValA;
//  
//  if (plus)    { counts++; }
//  if (minus)   { counts--; }
//  
//  if (valA != lastValA && valB != lastValB){
//    errors = 1;
//    
//  lastValA = valA;
//  lastValB = valB;
  
}

ISR(PCINT1_vect, ISR_ALIASOF(PCINT0_vect));
ISR(PCINT2_vect, ISR_ALIASOF(PCINT0_vect));
