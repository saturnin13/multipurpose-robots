#ifndef IRSENSOR_H
#define IRSENSOR_H

#include "Sensor.hpp"

class IRSensor : public Sensor {

  public:
    IRSensor(int ledPin, int ldrPin);

    void update();
    void reset();

    unsigned int value;

  private:
    const int ledPin;
    const int ldrPin;
};

#endif