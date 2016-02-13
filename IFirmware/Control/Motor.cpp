#include "Motor.h"
#include "Encoder.h"


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

uint16_t speedAutogear(uint16_t spd, int8_t isAccel, uint8_t pin){
  return GEAR3;
}

uint16_t dutyAutogear(uint16_t duty, int8_t isAccel, uint8_t pinPWM){
  return GEAR3;
}

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


void Motor::attachEncoder(uint8_t chA, uint8_t chB, uint8_t encoderNum){
  
  switch (encoderNum){
    case 1:
      m1_attachEncoder(chA, chB);
      break;

    case 2:
      m2_attachEncoder(chA, chB);
      break;
    
    default:
      return;
  }

  encNum = encoderNum;
  _isEncoder = true;
}


void Motor::update(){
  
  // Remember time stamp, and find out time elapsed since last call
  long int time;
  time = millis() - _lastTime;

  // If Encoder is in use, then get real feedback.
  if (_isEncoder) {

    // Get the counts till now
    long int count;
    if (encNum == 1) {
      count = m1_getCountsAndReset();
    }
    else if (encNum == 2) {
      count = m2_getCountsAndReset();
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

float Motor::getSpeed(){
  return currSpeed;
}

float Motor::getDistance(){
  return dist;
}

bool Motor::getDirection(){
  return isClockwise;
}

uint16_t Motor::getGear(){
  switch (_gear){
    case GEAR1: return 1;
    case GEAR2: return 2;
    case GEAR3: return 3;
    case GEAR4: return 4;
    default: return 0;
  }  
}

float Motor::getCurrent(){
  
}

float Motor::getVoltage(){
  
}

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
  if (isAutogear){ _gear = dutyAutogear(abs(_duty), isAccel, pinPWM); }
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


void Motor::setSpeed(int16_t rpm){
  
  // Find if motor needs to accelerate or decelerate
  int8_t isAccel;
  if      (abs(rpm) - abs(currSpeed) > 0) { isAccel =  1; }
  else if (abs(rpm) - abs(currSpeed) < 0) { isAccel = -1; }
  else                                    { isAccel =  0; }
  
  // Decide gear choice based on acceleration and speed.
  _gear = speedAutogear(currSpeed, isAccel, pinPWM);
  
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
