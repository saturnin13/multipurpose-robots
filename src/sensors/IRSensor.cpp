#include "IRSensor.hpp"
#include <Arduino.h>

// The minimum number of milliseconds that should pass
// between each sample.
#define SAMPLE_FREQUENCY 100

IRSensor::IRSensor(int pin) : pin(pin) {
    pinMode(pin, INPUT);
}

void IRSensor::update() {

    unsigned long now = millis();

    // Check if we can update the sensor data.
    if (now - lastUpdate < SAMPLE_FREQUENCY) {
        return;
    }

    auto ldrStatus = digitalRead(pin);

    this->obstacleDetected = (ldrStatus == LOW) ? true : false;
    this->lastUpdate = now;
}

void IRSensor::reset() {
    this->obstacleDetected = false;
    this->lastUpdate = 0;
}