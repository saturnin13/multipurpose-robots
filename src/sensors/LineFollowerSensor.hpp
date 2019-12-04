#ifndef LINEFOLLOWERSENSOR_H
#define LINEFOLLOWERSENSOR_H

#include "Sensor.hpp"
class LineFollowerSensor : public Sensor {

  public:
    LineFollowerSensor(int pin0, int pin1, int pin2, int pin3, int pin4);

    void update();
    void reset();

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
    
    int line0DetectionCount = 0;
    int line1DetectionCount = 0;
    int line2DetectionCount = 0;
    int line3DetectionCount = 0;
    int line4DetectionCount = 0;
  
};

#endif