#include "GlobalConfig.h"

void initializePins(){

  pinMode(MLeft_PWM, OUTPUT);
  pinMode(MLeft_IN1, OUTPUT);
  pinMode(MLeft_IN2, OUTPUT);
  
  pinMode(MRight_PWM, OUTPUT);
  pinMode(MRight_IN1, OUTPUT);
  pinMode(MRight_IN2, OUTPUT);
  
  pinMode(LED_IND1, OUTPUT);
  pinMode(LED_IND2, OUTPUT);
  pinMode(LED_IND3, OUTPUT);
  
  
  pinMode(MRight_DG, INPUT);
  pinMode(MLeft_DG, INPUT);

  pinMode(BATT_V, INPUT);
  pinMode(MRight_CS, INPUT);
  pinMode(MRight_DG, INPUT);

  pinMode(MLeft_CHA, INPUT);
  pinMode(MLeft_CHB, INPUT);
  pinMode(MRight_CHA, INPUT);
  pinMode(MRight_CHB, INPUT);
  
  pinMode(MLeft_VOL, INPUT);
  pinMode(MRight_VOL, INPUT);
}
