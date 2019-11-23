#ifndef MOTOR_H
#define MOTOR_H

#include "Actuator.hpp"

class Motor : public Actuator {
  public:
    Motor(int in1pin, int in2pin, int in3pin, int in4pin, int ena1pin, int ena2pin);

    void configure(bool forward, int stepTime, int numberSteps);
    void enact();

    bool workInProgress;

  private:
    const int in1Pin;
    const int in2Pin;
    const int in3Pin;
    const int in4Pin;

    int stepNumber;
    int numberSteps;
    bool clockwise;
    unsigned int stepTime;
    unsigned long lastUpdate;
};

#endif
