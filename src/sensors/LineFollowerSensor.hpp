#ifndef LINEFOLLOWERSENSOR_H
#define LINEFOLLOWERSENSOR_H

#include "Sensor.hpp"
#include "../Constants.hpp"

class LineFollowerSensor : public Sensor {

  public:
    LineFollowerSensor(int pin0, int pin1, int pin2, int pin3, int pin4);

    void update();
    void reset();

    //TODO change in line, edge or nothing, or just return value in addition
    bool lineDetected0 = false;
    bool lineDetected1 = false;
    bool lineDetected2 = false;
    bool lineDetected3 = false;
    bool lineDetected4 = false;

    bool unanimousDetection();

  private:
    int pin0;
    int pin1;
    int pin2;
    int pin3;
    int pin4;
};

#endif