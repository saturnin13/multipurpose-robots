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
    // unsigned int value1;
    // unsigned int value2;
    // unsigned int value3;
    // unsigned int value4;
    // unsigned int value5;

  private:
    int *pins;
    const unsigned int sensorCount;

    // const int ldr0Pin;
    // const int ldr1Pin;
    // const int ldr2Pin;
    // const int ldr3Pin;
    // const int ldr4Pin;
};

#endif