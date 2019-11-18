#ifndef ULTRASONICSENSOR_H
#define ULTRASONICSENSOR_H

#include "Sensor.hpp"

class UltrasonicSensor : public Sensor {

  public:
    UltrasonicSensor();

    void update();
    void reset();
};

#endif