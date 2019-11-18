#ifndef IRSENSOR_H
#define IRSENSOR_H

#include "Sensor.hpp"

class IRSensor : public Sensor {

  public:
    IRSensor();

    void update();
    void reset();
};

#endif