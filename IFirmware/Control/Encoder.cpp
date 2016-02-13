#include "Encoder.h"
#include "Arduino.h"

// Global Variables: Compiler shall not optimize these!
volatile uint8_t enc1_chA; /*!< Arduino pin connected to Channel A of Encoder 1  */
volatile uint8_t enc1_chB; /*!< Arduino pin connected to Channel B of Encoder 1  */
volatile uint8_t enc2_chA; /*!< Arduino pin connected to Channel A of Encoder 2  */
volatile uint8_t enc2_chB; /*!< Arduino pin connected to Channel B of Encoder 2  */

volatile long int enc1_counts; /*!< Counts on Encoder 1  \note: It is a 32-bit quantity */
volatile long int enc2_counts; /*!< Counts on Encoder 2  \note: It is a 32-bit quantity */

volatile int enc1_chA_lastVal; /*!< Last logical state of Channel A of Encoder 1  */
volatile int enc1_chB_lastVal; /*!< Last logical state of Channel B of Encoder 1  */ 
volatile int enc2_chA_lastVal; /*!< Last logical state of Channel A of Encoder 2  */
volatile int enc2_chB_lastVal; /*!< Last logical state of Channel B of Encoder 2  */
volatile int enc1_error; /*!< Errors on Encoder 1  */
volatile int enc2_error; /*!< Errors on Encoder 2  */


/*! 
  Stores the AVR based Register pointers for the given Arduino pin.
  
  \sa PoluluMotor Library by Abhishek N. Kulkarni (https://github.com/abhibp1993/PoluluMotor)
  \sa Polulu's OrangutanDigital Library.
*/
typedef struct IOStruct
{
	// if these aren't volatile, the compiler sometimes incorrectly optimizes away operations involving these registers:
	volatile unsigned char* pinRegister;
	volatile unsigned char* portRegister;
	volatile unsigned char* ddrRegister;
	unsigned char bitMask;
} IOPin;


/*! 
  Updates IOPin structure to contain the respective registers to corresponding Arduino pin.
  
  \param pin Arduino pin for which the registers need to be found out. 
  \param io Address of IOPin instance. 

  \sa hardware/arduino/cores/arduino/Arduino.h for understanding of implementation.
  \note
    In earlier version, the functions were implemented by hand. However, Arduino
    library has optimized the register access functions using macros to Flash Memory.  
*/
void getIORegisters(unsigned char pin, IOPin* io){
  uint8_t port = digitalPinToPort(pin);
  
  io->pinRegister = portInputRegister(port);
  io->portRegister = portOutputRegister(port);
  io->ddrRegister = portModeRegister(port);
  io->bitMask = digitalPinToBitMask(pin);
}
  
/*! 
  Configures all internal registers as required to configure the pin
  as Pin Change Interrupt. 

  \param pin Arduino pin on which the Interrupt (PCINT) needs to be enabled. 
  
  \sa hardware/arduino/cores/arduino/Arduino.h for understanding of implementation.
*/
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


/*! 
  Wrapper function to enable the Encoder 1.

  \param chA: The Arduino pin to which channel A of Encoder is connected.
  \param chB: The Arduino pin to which channel B of Encoder is connected.
  
*/
void enc1_attachEncoder(uint8_t chA, uint8_t chB){
  enc1_chA = chA;
  enc1_chB = chB;
  
  enableInterrupts(chA);
  enableInterrupts(chB);
}

/*! 
  Wrapper function to enable the Encoder 2.

  \param chA: The Arduino pin to which channel A of Encoder is connected.
  \param chB: The Arduino pin to which channel B of Encoder is connected.
  
*/
void enc2_attachEncoder(uint8_t chA, uint8_t chB){
  enc2_chA = chA;
  enc2_chB = chB;
  
  enableInterrupts(chA);
  enableInterrupts(chB);
}

  


/*!
  Returns the counts recorded on Encoder 1 till now from start. 
  \note: The Clockwise rotation of encoder results in incrementing of count, while the 
    CounterClockwise rotation of encoder results in decrementing. 
*/
long int enc1_getCounts(){
  cli();
  long int temp = enc1_counts;
  sei();
  return temp;
}

/*!
  Returns the counts recorded on Encoder 2 till now from start. 
  \note: The Clockwise rotation of encoder results in incrementing of count, while the 
    CounterClockwise rotation of encoder results in decrementing. 
*/
long int enc2_getCounts(){
  cli();
  long int temp = enc2_counts;
  sei();
  return temp;
}

/*!
  Returns the counts recorded on Encoder 1 since last call to this function
  and resets the counter to ZERO.
*/
long int enc1_getCountsAndReset(){
  cli();
  long int temp = enc1_counts;
  enc1_counts = 0;
  sei();
  return temp;
}

/*!
  Returns the counts recorded on Encoder 2 since last call to this function
  and resets the counter to ZERO.
*/
long int enc2_getCountsAndReset(){
  cli();
  long int temp = enc2_counts;
  enc2_counts = 0;
  sei();
  return temp;
}


/*!
  Implements the Interrupt Service Routine to asynchronously detect the transitions
  on the encoders and increment or decrement the count accordingly.
*/
ISR(PCINT0_vect){
    
  // Read present status of pin
  uint8_t enc1_chA_now = digitalRead(enc1_chA); //digitalRead(enc1_CHA);
  uint8_t enc1_chB_now = digitalRead(enc1_chB); //digitalRead(enc1_chB);
  uint8_t enc2_chA_now = digitalRead(enc2_chA); //digitalRead(enc2_chA);
  uint8_t enc2_chB_now = digitalRead(enc2_chB); //digitalRead(enc2_chB);
  
  
  // if transition occured on enc1, update enc1.
  if ((enc1_chA_now ^ enc1_chA_lastVal) || (enc1_chB_now ^ enc1_chB_lastVal)){
    
    // Update Count
    if (enc1_chB_now ^ enc1_chA_lastVal){ enc1_counts++; }
    if (enc1_chA_now ^ enc1_chB_lastVal){ enc1_counts--; }
    
    // Update error, if any
    if ((enc1_chA_now ^ enc1_chA_lastVal) && (enc1_chB_now ^ enc1_chB_lastVal)){ enc1_error = 1; }
    
    // update last value
    enc1_chA_lastVal = enc1_chA_now;
    enc1_chB_lastVal = enc1_chB_now;
  }
  
  // if transition occured on enc1, update count of enc1.
  if ((enc2_chA_now ^ enc2_chA_lastVal) || (enc2_chB_now ^ enc2_chB_lastVal)){
    
    // Update Count
    if (enc2_chB_now ^ enc2_chA_lastVal){ enc2_counts++; }
    if (enc2_chA_now ^ enc2_chB_lastVal){ enc2_counts--; }
    
    // Update error, if any
    if ((enc2_chA_now ^ enc2_chA_lastVal) && (enc2_chB_now ^ enc2_chB_lastVal)){ enc2_error = 1; }
    
    // update last value
    enc2_chA_lastVal = enc2_chA_now;
    enc2_chB_lastVal = enc2_chB_now;
  } 
}


ISR(PCINT1_vect, ISR_ALIASOF(PCINT0_vect)); /*!< PCINT1 aliased to execute the routine of PCINT0 */
ISR(PCINT2_vect, ISR_ALIASOF(PCINT0_vect)); /*!< PCINT2 aliased to execute the routine of PCINT0 */


