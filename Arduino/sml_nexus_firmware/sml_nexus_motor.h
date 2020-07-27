#include "Arduino.h"

//--------------------
// Nexus Motor object
//--------------------
class nexusMotor
{
  public:
    nexusMotor(uint8_t pin1, uint8_t pin2);
    void setSpeed(int16_t speed);
    
  protected:
    uint8_t _pin1;
    uint8_t _pin2;
};

nexusMotor::nexusMotor(uint8_t pin1, uint8_t pin2)
{
  _pin1 = pin1;
  _pin2 = pin2;
  
  pinMode(_pin1, OUTPUT);
  pinMode(_pin2, OUTPUT);
  
  digitalWrite(_pin1, LOW);
  digitalWrite(_pin2, LOW);
}

void nexusMotor::setSpeed(int16_t speed)
{
  // Make sure the speed is within the limit.
  if (speed > 255) {
    speed = 255;
  } else if (speed < -255) {
    speed = -255;
  }
  
  // Set the speed and direction.
  if (speed >= 0) {
    analogWrite(_pin1, speed);
    analogWrite(_pin2, 0);
  } else {
    analogWrite(_pin1, 0);
    analogWrite(_pin2, -speed);
  }
}
