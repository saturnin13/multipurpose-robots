#ifndef SENSOR_H
#define SENSOR_H

class Sensor {
  public:
    // Prompts the sensor to update itself.
    virtual void update() = 0;

    // Reset the internal state of the sensor.
    virtual void reset() = 0;
};

#endif