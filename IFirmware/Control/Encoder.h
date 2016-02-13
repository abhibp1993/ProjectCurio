#ifndef ENCODER_h
#define ENCODER_h

  #include "GlobalConfig.h"
  #include <avr/io.h>
  #include <avr/interrupt.h>
  
  #define CPR 1636.8   /*!< Counts per revolution of 25mm Diameter Polulu Motor with Encoder. */ 
  
  #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))  /*!< Clear bit MACRO. Defined for Optimization Purposes. */ 
  #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))   /*!< Set bit MACRO. Defined for Optimization Purposes. */ 
  

  // User Function Definitions  
  void enc1_attachEncoder(uint8_t chA, uint8_t chB);
  void enc2_attachEncoder(uint8_t chA, uint8_t chB);
  
  long int enc1_getCounts();
  long int enc2_getCounts();
  long int enc1_getCountsAndReset();
  long int enc2_getCountsAndReset();
  uint8_t  enc1_getErrors();
  uint8_t  enc2_getErrors();
  
#endif

