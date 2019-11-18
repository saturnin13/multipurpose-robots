#ifndef ULTRASONICSENSOR_H
#define ULTRASONICSENSOR_H

#include "Sensor.hpp"

class UltrasonicSensor : public Sensor {

  public:
    UltrasonicSensor(int triggerPin, int echoPin);

    void update();
    void reset();

    // The distance in millimeters measured by the sensor.
    unsigned long distance;

  private:
    const int triggerPin;
    const int echoPin;

    long microsecondsToMillimeters(long time);
};

#endif