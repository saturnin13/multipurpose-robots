#ifndef SENSOR_H
#define SENSOR_H

#include "../Constants.hpp"

class Sensor {
    public:
        // Prompts the sensor to update itself.
        virtual void update() = 0;

        // Reset the internal state_machine of the sensor.
        virtual void reset() = 0;

    protected:

        // The timestamp (in milliseconds) for when the sensor data was last updated.
        unsigned long lastUpdate = 0;

        // The delay for this  sensor
        unsigned long sampling_rate = DEFAULT_SAMPLING_RATE;
};

#endif