#ifndef LINEFOLLOWERSENSOR_H
#define LINEFOLLOWERSENSOR_H

#include "Sensor.hpp"

class LineFollowerSensor : public Sensor {

  public:
    LineFollowerSensor();

    void update();
    void reset();
};

#endif