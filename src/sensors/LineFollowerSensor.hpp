#ifndef LINEFOLLOWERSENSOR_H
#define LINEFOLLOWERSENSOR_H

#include "Sensor.hpp"

class LineFollowerSensor : public Sensor {

  public:
    LineFollowerSensor(int *pins, unsigned int sensorCount);
    ~LineFollowerSensor();

    void update();
    void reset();

    unsigned int *values;

  private:
    int *pins;
    const unsigned int sensorCount;
};

#endif