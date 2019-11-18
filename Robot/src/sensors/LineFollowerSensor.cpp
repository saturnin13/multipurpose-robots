#include "LineFollowerSensor.hpp"
#include <Arduino.h>

// The minimum number of milliseconds that should pass
// between each sample.
#define SAMPLE_FREQUENCY 100

LineFollowerSensor::LineFollowerSensor(int *pins, unsigned int sensorCount)
    : sensorCount(sensorCount) {
    this->values = new unsigned int[sensorCount];
    this->pins = new int[sensorCount];

    for (unsigned int i = 0; i < sensorCount; i++) {
        // Copy each pin and configure pin mode
        this->pins[i] = pins[i];
        pinMode(pins[i], INPUT);

        // Initialize value
        this->values[i] = 0;
    }
}

LineFollowerSensor::~LineFollowerSensor() {
    delete[] values;
    delete[] pins;
}

void LineFollowerSensor::update() {

    unsigned long now = millis();

    // Check if we can update the sensor data.
    if (now - lastUpdate < SAMPLE_FREQUENCY) {
        return;
    }

    for (unsigned int i = 0; i < this->sensorCount; i++) {
        this->values[i] = (unsigned int)analogRead(this->pins[i]);
    }
}

void LineFollowerSensor::reset() {

    for (unsigned int i = 0; i < this->sensorCount; i++) {
        this->values[i] = 0;
    }

    this->lastUpdate = 0;
}