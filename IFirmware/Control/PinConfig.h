#ifndef PINCONFIG_h
#define PINCONFIG_h

//==================================================================
  // Pin Configurations (assignments)

  #define M1_PWM  9      // Motor 1: PWM pin
  #define M1_IN1  6      // Motor 1: Direction Control Pin 1
  #define M1_IN2  7      // Motor 1: Direction Control Pin 2
  
  #define M2_PWM  10     // Motor 2: PWM pin
  #define M2_IN1  11     // Motor 2: Direction Control Pin 1
  #define M2_IN2  12     // Motor 2: Direction Control Pin 2
 
  #define M1_DG   A6     // Motor1 diagnostic failure 
  #define M2_DG   A7     // Motor2 diagnostic failure
  
  #define IN_STAT A5     // Indicator LED: Status of Processing
  #define IN_M1OK  8     // Indicator LED: M1 working ok => OFF, else ON
  #define IN_M2OK 13     // Indicator LED: M2 working ok => OFF, else ON
  
  #define BATT_V  A4     // Battery (LiPo) Voltage estimate
  #define M1_CS   A0     // M1: Current Sense
  #define M2_CS   A1     // M2: Current Sense
  
  #define M1_CHA  2      // M1: Encoder Channel A
  #define M1_CHB  3      // M1: Encoder Channel B

  #define M2_CHA  4      // M1: Encoder Channel A
  #define M2_CHB  5      // M2: Encoder Channel B
  
  #define M1_VOL  A2     // M1: Voltage estimate
  #define M2_VOL  A3     // M2: Voltage estimate
  
  
#endif
