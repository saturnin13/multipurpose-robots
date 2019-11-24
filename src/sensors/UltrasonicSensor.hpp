#ifndef ULTRASONICSENSOR_H
#define ULTRASONICSENSOR_H

#include "Sensor.hpp"
#include "NewPing.h"

class UltrasonicSensor : public Sensor {

  public:
    UltrasonicSensor(int triggerPin, int echoPin);

    void update();
    void reset();

    // The distance in centimeters measured by the sensor.
    // Set to 0 if 
    unsigned long distance;

  private:
    const int triggerPin;
    const int echoPin;
    NewPing sonar;

};

#endif