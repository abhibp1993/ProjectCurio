#ifndef MOTOR_h
#define MOTOR_h
  
  #include "Arduino.h"
  
  #define CPR   1636.8
  
  #define GEAR1 1024
  #define GEAR2  256
  #define GEAR3   64
  #define GEAR4    8
  
  #define MINDUTY  21
  #define MAXDUTY 220
  
  #define MAXDUTYCHANGE 0.5
  
  class Motor;  
  
  class Motor{
    private:
      uint8_t pinPWM, pinIN1, pinIN2;
      
      float currSpeed;
      float dist;
      bool isClockwise;
      
      bool _isEncoder;
      uint8_t encNum;
      long int _lastTime;
      int16_t _duty;
      int16_t _gear;
      int16_t _lastErr;
      int16_t _ITerm;
      
      void applyPWM(int16_t, uint16_t);
      
    public:
      float Kp, Ki, Kd;
      bool isAutogear;
      
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
