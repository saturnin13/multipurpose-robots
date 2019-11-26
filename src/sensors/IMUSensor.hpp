#ifndef IMUSENSOR_H
#define IMUSENSOR_H

#include "Sensor.hpp"

class IMUSensor : public Sensor {

  public:
    IMUSensor();

    void update();
    void reset();

    double xAngle, yAngle, zAngle;
};

#endif