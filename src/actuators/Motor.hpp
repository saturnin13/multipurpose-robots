#ifndef MOTOR_H
#define MOTOR_H

#include "Actuator.hpp"

class Motor : public Actuator {
  public:
    Motor(int in1Pin, int in2Pin, int enaPin);

    void configure(bool fordward, int speed);
    void enact();

    bool stopped;

  private:
    const int in1Pin;
    const int in2Pin;
    const int enaPin;

    bool forward;
    int pwmSpeed;
    unsigned long stopTime;
};

#endif
