#ifndef STEPPERMOTOR_H
#define STEPPERMOTOR_H

#include "Actuator.hpp"

class StepperMotor : public Actuator {
  public:
    StepperMotor(int in1pin, int in2pin, int in3pin, int in4pin);

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