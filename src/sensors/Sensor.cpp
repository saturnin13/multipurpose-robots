#include "Sensor.hpp"

void Sensor::update() {
    unsigned long now = millis();

    // Check if we can update the sensor data.
    if (now - lastUpdate < this->sampling_frequency) {
        return;
    }

    this->lastUpdate = now;
}