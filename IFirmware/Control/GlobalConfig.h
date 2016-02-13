#ifndef GLOBALCONFIG_h
#define GLOBALCONFIG_h
  
  #include "Arduino.h"
  
  // Offline PID Configuration
  #define OFFLINE_PID false  /*!< Configures if PID gains will be updated online via ROS Communication. 
                              \note Setting it to true will use the 
                                \link MLEFT_Kp \endlink ,
                                \link MLEFT_Ki \endlink ,
                                \link MLEFT_Kd \endlink ,
                                \link MRIGHT_Kp \endlink ,
                                \link MRIGHT_Ki \endlink ,
                                \link MRIGHT_Kd \endlink as gains */
  #define MLEFT_Kp  1.0     /*!< P-gain for Motor 1, if Offline PID is used. */
  #define MLEFT_Ki  0.0     /*!< I-gain for Motor 1, if Offline PID is used. */
  #define MLEFT_Kd  0.0     /*!< D-gain for Motor 1, if Offline PID is used. */
  #define MRIGHT_Kp 1.0     /*!< P-gain for Motor 2, if Offline PID is used. */
  #define MRIGHT_Ki 0.0     /*!< I-gain for Motor 2, if Offline PID is used. */
  #define MRIGHT_Kd 0.0     /*!< D-gain for Motor 2, if Offline PID is used. */

  // Battery Voltage Measurement
  #define BATTV_WINDOWSIZE 10   /*!< Number of samples to average for estimating battery voltage.  */
  #define BATTV_THRESHOLD  6.8  /*!< Minimum Voltage at which motors can be allowed to work on battery.  */
  
  // Motor Thresholds
  #define CURR_SOFT_THRESHOLD 4.5  /*!< Current at which warning should be issued that motors are overloading.  */
  #define CURR_HARD_THRESHOLD 6    /*!< Current at which further increase in current may damage motors.  */
  #define VOL_SOFT_THRESHOLD 6     /*!< Voltage at which warning should be issued that motors are overloading.  */
  
  
  // WARNINGS
  #define WARN_LEFTM_CURR    "W1002"
  #define WARN_RIGHTM_CURR   "W1003"
  #define WARN_LEFTM_VOL     "W1004"
  #define WARN_RIGHTM_VOL    "W1005"
  #define WARN_LEFTM_NOENCODER    "W1006"
  #define WARN_RIGHTM_NOENCODER   "W1007"
  
  // ERRORS
  #define ERR_BATTV          "E1001"
  #define ERR_LEFTM_CURR     "E1002"
  #define ERR_RIGHTM_CURR    "E1003"
  
  
  void initializePins();
    
//==================================================================
// Pin Configurations (assignments)

  #define MLeft_PWM      10      /*!< Motor 1: PWM pin  */
  #define MLeft_IN1      12      /*!< Motor 1: Direction Control Pin 1 */
  #define MLeft_IN2      11      /*!< Motor 1: Direction Control Pin 2 */
  
  #define MRight_PWM      9     /*!< Motor 2: PWM pin */
  #define MRight_IN1      8     /*!< Motor 2: Direction Control Pin 1 */
  #define MRight_IN2      7     /*!< Motor 2: Direction Control Pin 2 */
 
  #define MLeft_DG       A7     /*!< Motor1 diagnostic failure  */
  #define MRight_DG       4     /*!< Motor2 diagnostic failure */
  
  #define LED_IND1       A3     /*!< Indicator LED: Color?? */
  #define LED_IND2        3     /*!< Indicator LED: Color?? */
  #define LED_IND3        2     /*!< Indicator LED: Color?? */
  
  #define BATT_V         A5     /*!< Battery (LiPo) Voltage estimate  */
  #define MLeft_CS       A6     /*!< M1: Current Sense */
  #define MRight_CS      A4     /*!< M2: Current Sense */
  
  #define MLeft_CHA      A2     /*!< M1: Encoder Channel A */
  #define MLeft_CHB      13     /*!< M1: Encoder Channel B */

  #define MRight_CHA      5      /*!< M1: Encoder Channel A */
  #define MRight_CHB      6      /*!< M2: Encoder Channel B */
  
  #define MLeft_VOL      A1     /*!< M1: Voltage estimate */
  #define MRight_VOL     A0     /*!< M2: Voltage estimate */
  
#endif
