#ifndef MOTOR_h
#define MOTOR_h
  
  #include "Arduino.h" 
  
  #define GEAR1 1024     /*!< Gear 1 ==> PWM Frequency of 30.5175Hz */
  #define GEAR2  256     /*!< Gear 2 ==> PWM Frequency of 122.075Hz */
  #define GEAR3   64     /*!< Gear 2 ==> PWM Frequency of 488.281Hz */
  #define GEAR4    8     /*!< Gear 3 ==> PWM Frequency of 3906.25Hz */
  
  #define MINDUTY  21    /*!< Minimum Duty Cycle which may be applied to motor. 
                            \remark The value is found using characteristic of motor at No-Robot Load in GEAR1 */
  #define MAXDUTY 220    /*!< Maximum Duty Cycle which may be applied to motor. 
                            \remark The value is chosen so as to restrict maximum voltage to be applied 
                             to motor below voltage mentioned in specifications. */  
  #define MAXDUTYCHANGE 0.5  /*!< Maximum Duty Cycle Change permited during one step */
  

  class Motor;  
  class Motor{
    private:
      uint8_t pinPWM;      /*!< Arduino pin connected to PWM pin of Motor Driver */
      uint8_t pinIN1;      /*!< Arduino pin connected to IN1 pin of Motor Driver */
      uint8_t pinIN2;      /*!< Arduino pin connected to IN2 pin of Motor Driver */
      
      float currSpeed;     /*!< Current Speed of Motor 
                              \remark: currSpeed is updated through \link update() \endlink function */
      float dist;          /*!< Number of Rotations completed by shaft since last call to \link update() \endlink 
                              \remark: dist is updated through \link update() \endlink function */
      bool isClockwise;    /*!< Direction of rotation of motor shaft. 
                              \remark: isClockwise is updated through \link update() \endlink function */
      
      bool _isEncoder;     /*!< Whether Encoder is connected or not. \sa \link WARN_LEFTM_NOENCODER \endlink, 
                                \link WARN_RIGHTM_NOENCODER \endlink. */
      uint8_t encNum;      /*!< Value indicating whether Encoder 1 or 2 is connected to this motor. */
      long int _lastTime;  /*!< ??? */
      int16_t _duty;       /*!< Duty Cycle of PWM applied to motor */
      int16_t _gear;       /*!< Frequency of PWM applied to motor */
      int16_t _lastErr;    /*!< Last Error in speed as computed in PID Routine. \sa \link setSpeed() \endlink. */
      int16_t _ITerm;      /*!< Integral Error in speed as computed in PID Routine. \sa \link setSpeed() \endlink. */
      
      void applyPWM(int16_t, uint16_t); 
      
    public:
      float Kp;    /*!< P-gain of PID controller for Motor. */
      float Ki;    /*!< I-gain of PID controller for Motor. */
      float Kd;    /*!< D-gain of PID controller for Motor. */
      bool isAutogear;  /*!< Flag indicating whether gears (frequency) of PWM is to 
                           be chosen automatically or manually given.
                          \note setting isAutogear = true/false has no effect on \link setSpeed \endlink function.
                           setSpeed internally uses autogear.  */     
                           
      Motor(uint8_t, uint8_t, uint8_t);  
      void attachEncoder(uint8_t, uint8_t, uint8_t);
      
      void setSpeed(int16_t);
      void setPWM(float, uint16_t = GEAR1);
      
      void update();
      float getSpeed();
      float getCurrent();
      bool getDirection();
      float getDistance();
      float getVoltage();
      uint16_t getGear();
      
  };
#endif
