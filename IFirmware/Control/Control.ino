#include "GlobalConfig.h"
#include "Motor.h"


// Global Communication Variables
bool errOccurred = false, isRPM = true;
float batteryVoltage; 
float mLeftRef, mRightRef;
float mLeftSpeed, mRightSpeed; 
float mLeftCurrent, mRightCurrent; 
float mLeftDistance, mRightDistance;
float mLeftVoltage, mRightVoltage;
uint16_t gear;
char strWarnErr[50];
uint8_t ptrWarnErr;

// Motor Instances
Motor mLeft(MLeft_PWM, MLeft_IN1, MLeft_IN2);
Motor mRight(MRight_PWM, MRight_IN1, MRight_IN2);


float getBatteryVoltage(){
  uint16_t input;
  for (int i = 0; i < BATTV_WINDOWSIZE; i++){
    input += analogRead(BATT_V);
  }
  
  // Scale it appropriately to actual voltage in Volts and return
  // To Do.
}

void reportError(char errStr[]){
  uint8_t i = 0;
  while (errStr[i] != '\0'){
    *(strWarnErr + ptrWarnErr) = errStr[i];
    i++;
    ptrWarnErr++;
  }  
}

void setup() {
  // ROS Node Initialization
  // To Do
  
  // Advertise Publishers and Subscribers
  // To Do 
  
  // Initialize Pin Configuration
  initializePins();
  
  // Motor: Attach Encoders
  mLeft.attachEncoder(MLeft_CHA, MLeft_CHB, 1);
  mRight.attachEncoder(MRight_CHA, MRight_CHB, 2);

  // If offline PID is required, set the gains
  #if (OFFLINE_PID == true)
    mLeft.Kp = MLEFT_Kp;
    mLeft.Ki = MLEFT_KI;
    mLeft.Kd = MLEFT_KD;
    
    mRight.Kp = MRIGHT_Kp;
    mRight.Ki = MRIGHT_Ki;
    mRight.Kd = MRIGHT_Kd;
  #endif
  
  // INDICATOR 3: GREEN
  digitalWrite(LED_IND3, HIGH);
}

void loop() {
  
  // Check Battery Voltage
  batteryVoltage = getBatteryVoltage();
  if (batteryVoltage < BATTV_THRESHOLD){
    // REPORT ERROR 
    errOccurred = true;
    reportError(ERR_BATTV);
  }
  
  // If no error till now... then update variables
  if (!errOccurred){
    
    // Update motor parameters
    mLeft.update();
    mRight.update();
    
    // Update Variables
    mLeftSpeed = mLeft.getSpeed();
    mRightSpeed = mRight.getSpeed();
    
    mLeftCurrent = mLeft.getCurrent();
    mRightCurrent = mRight.getCurrent(); 
    
    mLeftDistance = mLeft.getDistance();
    mRightDistance = mRight.getDistance();
    
    mLeftVoltage = mLeft.getVoltage();
    mRightVoltage = mRight.getVoltage(); 
    
    gear = mLeft.getGear();
    
    // Check for protection
    if (mLeftCurrent > CURR_SOFT_THRESHOLD){ reportError(WARN_LEFTM_CURR); }
    else if (mLeftCurrent > CURR_HARD_THRESHOLD){ reportError(ERR_LEFTM_CURR); errOccurred = true; }
    
    if (mRightCurrent > CURR_SOFT_THRESHOLD){ reportError(WARN_RIGHTM_CURR); }
    else if (mRightCurrent > CURR_HARD_THRESHOLD){ reportError(ERR_RIGHTM_CURR); errOccurred = true; }

    if (mLeftVoltage > VOL_SOFT_THRESHOLD){ reportError(WARN_LEFTM_VOL); }
    if (mRightVoltage > VOL_SOFT_THRESHOLD){ reportError(WARN_RIGHTM_VOL); }
  }
  // else: all the values should be treated as garbage
  
  // If no error till now... then update variables
  if (!errOccurred){
    if (isRPM){
      mLeft.setSpeed(mLeftRef);
      mRight.setSpeed(mRightRef);
    }
    else{
      mLeft.isAutogear = true;
      mRight.isAutogear = true;
      
      mLeft.setPWM(mLeftRef);
      mRight.setPWM(mRightRef);
    }
  }
  else{
    mLeft.isAutogear = false;
    mRight.isAutogear = false;
    
    mLeft.setSpeed(0);
    mRight.setSpeed(0);
  }
  
  // ROS Communication 
  // To Do
  
  // Book keeping
  ptrWarnErr = 0;
}
