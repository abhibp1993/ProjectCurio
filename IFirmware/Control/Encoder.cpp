#include "Encoder.h"
#include "Arduino.h"

// Global Variables: Compiler shall not optimize these!
volatile uint8_t m1_chA, m1_chB, m2_chA, m2_chB;
volatile long int m1_counts, m2_counts;
volatile int m1_chA_lastVal, m1_chB_lastVal;
volatile int m2_chA_lastVal, m2_chB_lastVal;
volatile int m1_error, m2_error;


/* [HELPER]**********************************************************************
Structure: IOStruct 

Description:
  Stores the AVR based Register pointers for the given pin
  
Refer PoluluMotor Library by Abhishek N. Kulkarni (abhibp1993)
Refer to Polulu's OrangutanDigital Library.
**********************************************************************************/
typedef struct IOStruct
{
	// if these aren't volatile, the compiler sometimes incorrectly optimizes away operations involving these registers:
	volatile unsigned char* pinRegister;
	volatile unsigned char* portRegister;
	volatile unsigned char* ddrRegister;
	unsigned char bitMask;
} IOPin;


/* [HELPER]**********************************************************************
Function: getIORegisters 
Parameters: 
  1. pin: pin for which the registers need to be found out. 
  2. io: IOPin instance. 

Description:
  The function modifies the IOPin structure to contain the respective registers. 
  
Remark:
  In earlier version, the functions were implemented by hand. However, Arduino
  library has optimized the register access functions using macros to Flash Memory.

Refer to hardware/arduino/cores/arduino/Arduino.h for understanding.
**********************************************************************************/
void getIORegisters(unsigned char pin, IOPin* io){
  uint8_t port = digitalPinToPort(pin);
  
  io->pinRegister = portInputRegister(port);
  io->portRegister = portOutputRegister(port);
  io->ddrRegister = portModeRegister(port);
  io->bitMask = digitalPinToBitMask(pin);
}
  
/* [HELPER]**********************************************************************
Function: enableInterrupts 
Parameters: 
  1. pin: pin on which the PCINT needs to be enabled. 

Description:
  The function configures all internal registers as required to configure the pin
  as Pin Change Interrupt. 
  
Refer to hardware/arduino/cores/arduino/Arduino.h for understanding.
**********************************************************************************/
void enableInterrupts(uint8_t pin){

  // Disable Interrupts while configuration is in progress
  cli();
  
  // Fill the structure with appropriate ports and pins
  IOPin io;
  getIORegisters(pin, &io);
  
  // Enable Interrupt Mask
  if (io.pinRegister == &PINB)
    PCMSK0 |= io.bitMask; 
  if (io.pinRegister == &PINC)
    PCMSK1 |= io.bitMask;
  if (io.pinRegister == &PIND)
    PCMSK2 |= io.bitMask;
  
  // Set Data Direction Register
  cbi(*io.ddrRegister, io.bitMask);
  
  // Enable PCINT interrupt
  PCICR = 0x07;
  
  // Disable Interrupts while configuration is in progress
  sei();
}


/* [PUBLIC]**********************************************************************
Function: enableInterrupts 
Parameters: 
  1. chA: The Arduino pin to which channel A of Encoder is connected.
  2. chB: The Arduino pin to which channel B of Encoder is connected.

Description:
  Wrapper function to enable the Encoders on Motor 1.
  
**********************************************************************************/
void m1_attachEncoder(uint8_t chA, uint8_t chB){
  m1_chA = chA;
  m1_chB = chB;
  
  enableInterrupts(chA);
  enableInterrupts(chB);
}

/* [PUBLIC]**********************************************************************
Function: enableInterrupts 
Parameters: 
  1. chA: The Arduino pin to which channel A of Encoder is connected.
  2. chB: The Arduino pin to which channel B of Encoder is connected.

Description:
  Wrapper function to enable the Encoders on Motor 2.
  
**********************************************************************************/
void m2_attachEncoder(uint8_t chA, uint8_t chB){
  m2_chA = chA;
  m2_chB = chB;
  
  enableInterrupts(chA);
  enableInterrupts(chB);
}

  


/* [PUBLIC]**********************************************************************
Function: m1_getCounts
Parameters: None

Description:
  The function returns the counts recorded on Motor 1 till now.
  
**********************************************************************************/
long int m1_getCounts(){
  cli();
  long int temp = m1_counts;
  sei();
  return temp;
}

/* [PUBLIC]**********************************************************************
Function: m2_getCounts
Parameters: None

Description:
  The function returns the counts recorded on Motor 2 till now.
  
**********************************************************************************/
long int m2_getCounts(){
  cli();
  long int temp = m2_counts;
  sei();
  return temp;
}

/* [PUBLIC]**********************************************************************
Function: m1_getCountsAndReset
Parameters: None

Description:
  The function returns the counts recorded on Motor 1 till now and resets the 
  counter to ZERO.
  
**********************************************************************************/
long int m1_getCountsAndReset(){
  cli();
  long int temp = m1_counts;
  m1_counts = 0;
  sei();
  return temp;
}

/* [PUBLIC]**********************************************************************
Function: m2_getCountsAndReset
Parameters: None

Description:
  The function returns the counts recorded on Motor 2 till now and resets the 
  counter to ZERO.
  
**********************************************************************************/
long int m2_getCountsAndReset(){
  cli();
  long int temp = m2_counts;
  m2_counts = 0;
  sei();
  return temp;
}


// Interrupt Handling Routine
long int time = 0; 
ISR(PCINT0_vect){
  
  // Check if ISR is called properly.
  time = micros();
  
  // Read present status of pin
  uint8_t m1_chA_now = digitalRead(m1_chA); //digitalRead(M1_CHA);
  uint8_t m1_chB_now = digitalRead(m1_chB); //digitalRead(M1_CHB);
  uint8_t m2_chA_now = digitalRead(m2_chA); //digitalRead(M2_CHA);
  uint8_t m2_chB_now = digitalRead(m2_chB); //digitalRead(M2_CHB);
  
  
  // if transition occured on M1, update M1.
  if ((m1_chA_now ^ m1_chA_lastVal) || (m1_chB_now ^ m1_chB_lastVal)){
    
    // Update Count
    if (m1_chB_now ^ m1_chA_lastVal){ m1_counts++; }
    if (m1_chA_now ^ m1_chB_lastVal){ m1_counts--; }
    
    // Update error, if any
    if ((m1_chA_now ^ m1_chA_lastVal) && (m1_chB_now ^ m1_chB_lastVal)){ m1_error = 1; }
    
    // update last value
    m1_chA_lastVal = m1_chA_now;
    m1_chB_lastVal = m1_chB_now;
  }
  
  // if transition occured on M1, update count of M1.
  if ((m2_chA_now ^ m2_chA_lastVal) || (m2_chB_now ^ m2_chB_lastVal)){
    
    // Update Count
    if (m2_chB_now ^ m2_chA_lastVal){ m2_counts++; }
    if (m2_chA_now ^ m2_chB_lastVal){ m2_counts--; }
    
    // Update error, if any
    if ((m2_chA_now ^ m2_chA_lastVal) && (m2_chB_now ^ m2_chB_lastVal)){ m2_error = 1; }
    
    // update last value
    m2_chA_lastVal = m2_chA_now;
    m2_chB_lastVal = m2_chB_now;
  }
 
   time = micros() - time; 
}

// Alias other PCINTs to execute the routine of PCINT0
ISR(PCINT1_vect, ISR_ALIASOF(PCINT0_vect));
ISR(PCINT2_vect, ISR_ALIASOF(PCINT0_vect));


