#ifndef SENSOR_H
#define SENSOR_H

class Sensor {
  public:
    // Prompts the sensor to update itself.
    virtual void update() = 0;

    // Reset the internal state machine of the sensor.
    virtual void reset() = 0;

    // The timestamp (in milliseconds) for when the sensor data was last updated.
    unsigned long lastUpdate = 0;

};

#endif