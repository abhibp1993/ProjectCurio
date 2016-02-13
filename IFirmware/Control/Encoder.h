#ifndef ENCODER_h
#define ENCODER_h
  
  extern long int time;

  #include "GlobalConfig.h"
  #include <avr/io.h>
  #include <avr/interrupt.h>
  
  #define CPR 1636.8
  
  #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))    // defined for optimization purposes
  #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))     // defined for optimization purposes
  

  // User Function Definitions  
  void m1_attachEncoder(uint8_t chA, uint8_t chB);
  void m2_attachEncoder(uint8_t chA, uint8_t chB);
  
  long int m1_getCounts();
  long int m2_getCounts();
  long int m1_getCountsAndReset();
  long int m2_getCountsAndReset();
  uint8_t  m1_getErrors();
  uint8_t  m2_getErrors();
  
#endif

