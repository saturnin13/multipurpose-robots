#ifndef IRSENSOR_H
#define IRSENSOR_H

#include "Sensor.hpp"

class IRSensor : public Sensor {

  public:
    IRSensor(int pin);

    void update();
    void reset();

    bool lineDetected = false;

  private:
    const int pin;
};

#endif