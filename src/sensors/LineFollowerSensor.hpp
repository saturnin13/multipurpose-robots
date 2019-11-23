#ifndef LINEFOLLOWERSENSOR_H
#define LINEFOLLOWERSENSOR_H

#include "Sensor.hpp"

class LineFollowerSensor : public Sensor {

  public:
    LineFollowerSensor(int pin0, int pin1, int pin2, int pin3, int pin4);

    void update();
    void reset();

    unsigned int value0 = 0;
    unsigned int value1 = 0;
    unsigned int value2 = 0;
    unsigned int value3 = 0;
    unsigned int value4 = 0;

    bool unanimousDetection();

  private:
    int pin0;
    int pin1;
    int pin2;
    int pin3;
    int pin4;
};

#endif