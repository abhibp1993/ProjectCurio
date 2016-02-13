#include "Motor.h"
#include "Encoder.h"

/*!
  The function changes the frequency on pwm pin. 
 
  \param pin: PWM pin on which the frequency is to be set. 
  \param freqMacro: prescaling factor to set the frequency. 
        (USE \link GEAR1 \endlink, \link GEAR2 \endlink, \link GEAR3 \endlink, \link GEAR4 \endlink)

  \sa \li PoluluMotor Library by Abhishek N. Kulkarni (abhibp1993)
  \remark Adopted from macegr's code. (Post: http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1235060559/0#4)
  
  \note Currently only pin 9, 10 frequency can be changed. \n
        Changing frequency on either pin changes the pwm-frequency on other also. 
*/
void setPWMFrequency(uint8_t pin, int16_t freqMacro) {
  byte mode;
  if (pin == 9 || pin == 10) {
    switch (freqMacro) {
      case GEAR4: mode = 0x02; break;
      case GEAR3: mode = 0x03; break;
      case GEAR2: mode = 0x04; break;
      case GEAR1: mode = 0x05; break;
      default: return;
    }
    TCCR1B = TCCR1B & 0b11111000 | mode;
  }
}


/*!
  Returns the next gear (frequency of PWM) that should be used for smooth motor operation.
 
  \param spd: current speed of motor in RPM 
  \param isAccel: if the speed is to be increased (+1), maintained (0) or decreased (-1)
  \param currGear: Gear that is presently in use (USE \link GEAR1 \endlink, 
      \link GEAR2 \endlink, \link GEAR3 \endlink, \link GEAR4 \endlink)
  \param pin: Arduino pin on which the gear (frequency) is to be changed. 

  \remark pin parameter becomes essential as change of frequency on one pin of a timer, 
    results in change of frequency on other pin too. Hence, we choose the lower frequency 
    amongst the two. 
*/
uint16_t speedAutogear(uint16_t spd, uint16_t currGear, int8_t isAccel, uint8_t pin){

  if (spd == 0){ return GEAR1; }

  switch (currGear){
    case GEAR1:
      if      (isAccel == 1 && spd > 0 && spd <= 50)  {return GEAR4; }
      else if (isAccel == 1 && spd > 50)              {return GEAR3; }
      else                                            {return GEAR1; }      
      
    case GEAR2:
      if      (isAccel == 1 && spd > 0 && spd <= 100) {return GEAR2; }
      else if (isAccel == 1 && spd > 100)             {return GEAR3; }
      else if (isAccel == 0)                          {return GEAR2; }
      else                                            {return GEAR3; }
    
    case GEAR3:
      if      (isAccel == 1 && spd > 0 && spd <= 100) {return GEAR2; }
      else if (isAccel == 1 && spd > 100)             {return GEAR3; }
      else                                            {return GEAR3; }
    
    case GEAR4:
      return GEAR4;      
  }
  
}

/*!
  Returns the next gear (frequency of PWM) that should be used for smooth motor operation.
 
  \param duty: current duty cycle applied to motor in range (0-255)
  \param isAccel: if the speed is to be increased (+1), maintained (0) or decreased (-1)
  \param currGear: Gear that is presently in use (USE \link GEAR1 \endlink, 
      \link GEAR2 \endlink, \link GEAR3 \endlink, \link GEAR4 \endlink)
  \param pin: Arduino pin on which the gear (frequency) is to be changed. 

  \remark pin parameter becomes essential as change of frequency on one pin of a timer, 
    results in change of frequency on other pin too. Hence, we choose the lower frequency 
    amongst the two. 
*/
uint16_t dutyAutogear(uint16_t duty, uint16_t currGear, int8_t isAccel, uint8_t pinPWM){
  
  switch (currGear){
    case GEAR1:
      if      (duty >= 0 && duty < 26) { return GEAR1; }
      else if (duty > 255)             { return GEAR1; }
      else                             { return GEAR3; }
  
    case GEAR2:
      if      (isAccel == 1 && duty > 0 && duty < 49) { return GEAR1; }
      else if (isAccel != 1 && duty > 0 && duty < 49) { return GEAR2; }
      else                                            { return GEAR3; }
      
    case GEAR3:
      if      (isAccel == 1 && duty > 0 && duty < 26)   { return GEAR1; }
      else if (isAccel == 1 && duty >= 26 && duty < 55) { return GEAR3; }
      else if (isAccel != 1 && duty > 0 && duty < 55)   { return GEAR2; }
      else                                              { return GEAR3; }
      
    case GEAR4:
      if      (isAccel == 1 && duty > 0 && duty < 64) { return GEAR1; }
      else if (isAccel != 1 && duty > 0 && duty < 64) { return GEAR3; }
      else                                            { return GEAR4; }
  }
}



/*!
  Constructor of Motor class. Initializes internal class variables. 
  
  \param pwm: Arduino pin to which the PWM pin of motor driver is connected
  \param in1: Arduino pin to which the PWM pin of motor driver is connected
  \param in2: Arduino pin to which the PWM pin of motor driver is connected
*/
Motor::Motor(uint8_t pwm, uint8_t in1, uint8_t in2){
  
  // Control Pins
  pinPWM = pwm;
  pinIN1 = in1;
  pinIN2 = in2;
  
  // Initialize Internal Variables
  _isEncoder = false;
  _ITerm = 0;
  _lastErr = 0;
  _duty = 0;
}


/*!
  Attaches an encoder to the motor for speed feedback. 
  
  \param chA: Arduino pin to which the CHA pin of encoder is connected
  \param chB: Arduino pin to which the CHB pin of encoder is connected
  \param encoderNum: Which of encoder 1 or 2 should be used for this motor. 
      Choose between 1 or 2 only as only 2 encoders are available in \link Encoder.h \endlink.
*/
void Motor::attachEncoder(uint8_t chA, uint8_t chB, uint8_t encoderNum){
  
  switch (encoderNum){
    case 1:
      enc1_attachEncoder(chA, chB);
      break;

    case 2:
      enc2_attachEncoder(chA, chB);
      break;
    
    default:
      return;
  }

  encNum = encoderNum;
  _isEncoder = true;
}

/*!
  Updates state variables for motor. This includes speed, distance, and direction.
  
  \note: The function generates warning if encoder is not enabled, that is a successful
    call to \link Motor::attachEncoder \endlink has not been made. 
*/
void Motor::update(){
  
  // Remember time stamp, and find out time elapsed since last call
  long int time;
  time = millis() - _lastTime;

  // If Encoder is in use, then get real feedback.
  if (_isEncoder) {

    // Get the counts till now
    long int count;
    if (encNum == 1) {
      count = enc1_getCountsAndReset();
    }
    else if (encNum == 2) {
      count = enc2_getCountsAndReset();
    }
    else {
      return;
    }
    _lastTime = millis();

    // Compute the state variables: Direction
    if (count < 0) {
      isClockwise = false;
    }
    else{
      isClockwise = true;
    }

    // Compute the state variables: Distance
    dist = (float)(count) / CPR;

    // Compute the state variables: Speed
    currSpeed = (dist * 60000) / time;
  }
  else {
    // Raise Warning
    // Compute speed by duty and lookup table
  }
}

/*!
  Returns speed of motor in RPM.

  \note: This function does not compute anything. It is a wrapper function. For proper working of this function, it 
  is necessary to make a prior call to \link Motor::update() \endlink function.
*/
float Motor::getSpeed(){
  return currSpeed;
}

/*!
  Returns number of revolutions that motor has undergone since last call to \link Motor::update() \endlink function.

  \note: This function does not compute anything. It is a wrapper function. For proper working of this function, it 
  is necessary to make a prior call to \link Motor::update() \endlink function.
*/
float Motor::getDistance(){
  return dist;
}

/*!
  Returns TRUE if motor is rotating in Clockwise direction, else FALSE.

  \note: This function does not compute anything. It is a wrapper function. For proper working of this function, it 
  is necessary to make a prior call to \link Motor::update() \endlink function.
*/
bool Motor::getDirection(){
  return isClockwise;
}

/*!
  Returns the numeric value of gear in which the motor is being run. 
  \li GEAR1 - 1
  \li GEAR2 - 2
  \li GEAR3 - 3
  \li GEAR4 - 4
  
  \note: This function does not compute anything. It is a wrapper function. For proper working of this function, it 
  is necessary to make a prior call to \link Motor::setPWM() \endlink OR \link Motor::setSpeed() \endlink function.
  \n If the value returned is 0, It means gear (frequency) setting is not happening properly.
*/
uint16_t Motor::getGear(){
  switch (_gear){
    case GEAR1: return 1;
    case GEAR2: return 2;
    case GEAR3: return 3;
    case GEAR4: return 4;
    default: return 0;
  }  
}


/*!
  Returns the current drawn by motor in Amps.

  \note: This function does not compute anything. It is a wrapper function. For proper working of this function, it 
  is necessary to make a prior call to \link Motor::update() \endlink function.
*/ 
float Motor::getCurrent(){
  
}

/*!
  Returns the Voltage applied across the motor in Volt.

  \note: This function does not compute anything. It is a wrapper function. For proper working of this function, it 
  is necessary to make a prior call to \link Motor::update() \endlink function.
*/ 
float Motor::getVoltage(){
  
}

/*!
  Sets the frequency of PWM waveform according to gear specified and applies the duty cycle.
  
  \param duty: A value between 0-255 
  \param gear: Gear(frequency) that should be used (USE \link GEAR1 \endlink, 
      \link GEAR2 \endlink, \link GEAR3 \endlink, \link GEAR4 \endlink)
      
  \note: This function does not implement any protections while applying the duty and frequency. 
*/
void Motor::applyPWM(int16_t duty, uint16_t gear){
  
  // Set the frequency of PWM according to said gear.
  _gear = gear;
  setPWMFrequency(pinPWM, gear);
  
  // Free Run
  if (abs(duty) > 255) {
    pinMode(pinPWM, INPUT);
    return;
  }
  
  // Braking
  else if (abs(duty) == 0) {
    pinMode(pinPWM, OUTPUT);
    digitalWrite(pinIN1, HIGH);
    digitalWrite(pinIN2, HIGH);
    return;
  }
  
  // Run Mode
  else {
    pinMode(pinPWM, OUTPUT);
    digitalWrite(pinIN1,  (duty > 0));
    digitalWrite(pinIN2, !(duty > 0));
    analogWrite(pinPWM, abs(duty));
  }
  
}

/*!
  Sets the duty cycle of applied PWM waveform to motor. The frequency of PWM may 
  be changed by changing the gear. The function may be used in 2 ways. Firstly, 
  if the \link isAutogear \endlink is set to "False", the input parameter "gear"
  is used to set the frequency. However, if the \link isAutogear \endlink is "True"
  then even if gear argument is passed - it is ignored and gear is chosen using 
  \link dutyAutogear() \endlink function. 
  
  \param perDuty: A value between 0-1 
  \param gear: Gear(frequency) that should be used (USE \link GEAR1 \endlink, 
      \link GEAR2 \endlink, \link GEAR3 \endlink, \link GEAR4 \endlink) The default 
      value is GEAR1.
      
  \sa \link Motor::applyPWM() \endlink, \link Motor::applySpeed() \endlink
*/
void Motor::setPWM(float perDuty, uint16_t gear){
  
  int myDuty = 255 * perDuty;
  
  // Constrain the voltage applied to motor  
  if (abs(myDuty) < MINDUTY && abs(myDuty) > 0)     { myDuty = 2.0; }            // 2.0 => free run
  if (abs(myDuty) > MAXDUTY && abs(myDuty) <= 1.0)  { myDuty = MAXDUTY; } 
  
  // Determine if we are accelerating or decelerating
  int8_t isAccel;
  if      (abs(myDuty) - abs(_duty) > 0) { isAccel =  1; }
  else if (abs(myDuty) - abs(_duty) < 0) { isAccel = -1; }
  else                                   { isAccel =  0; }
  
  // Choose gear, i.e. frequency of PWM (manual or auto)
  if (isAutogear){ _gear = dutyAutogear(abs(_duty), _gear, isAccel, pinPWM); }
  else           { _gear = gear; }
  
  // Direction Reversal with large difference => Brake and Turn
  if ((myDuty ^ _duty < 0) && (abs(myDuty - _duty) > MAXDUTYCHANGE)){
    applyPWM(0, _gear);
    delay(1);
  }
  
  // apply PWM
  _duty = myDuty;
  applyPWM(myDuty, _gear);  
  
}

/*!
  Sets the speed of motor. \n The speed is regulated using a PID controller 
  which may be tuned using the \link Motor::Kp \endlink, \link Motor::Ki \endlink, 
  \link Motor::Kd \endlink variables. 
  
  \param rpm: A value between 0-\link MAXSPEED \endlink
  \sa \link Motor::applyPWM() \endlink, \link Motor::applySpeed() \endlink
*/
void Motor::setSpeed(int16_t rpm){
  
  // Find if motor needs to accelerate or decelerate
  int8_t isAccel;
  if      (abs(rpm) - abs(currSpeed) > 0) { isAccel =  1; }
  else if (abs(rpm) - abs(currSpeed) < 0) { isAccel = -1; }
  else                                    { isAccel =  0; }
  
  // Decide gear choice based on acceleration and speed.
  _gear = speedAutogear(currSpeed, _gear, isAccel, pinPWM);
  
  // PID Autocorrect
  int16_t err = rpm - currSpeed;
  _ITerm += err;
  int16_t diffErr = err - _lastErr;
  
  uint16_t correction = Kp * err + Ki * _ITerm + Kd * diffErr;
  _duty += correction;

  if (_duty > 0) {_duty = constrain(_duty, MINDUTY, MAXDUTY); }
  if (_duty < 0) {_duty = constrain(_duty, -MAXDUTY, -MINDUTY); }
  
  // Apply PWM Duty.
  isAutogear = false;
  setPWM((float)(_duty)/255, _gear);
  
  Serial.print("duty: "); Serial.println(_duty); 
}
