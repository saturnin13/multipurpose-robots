#include "IRSensor.hpp"
#include <Arduino.h>

// Sampling rate in milliseconds
#define SAMPLING_RATE 200

IRSensor::IRSensor(int pin) : pin(pin) {
    pinMode(pin, INPUT);
}

void IRSensor::update() {
    unsigned long now = millis();

    // Check if we can update the sensor data.
    if (now - this->lastUpdate < SAMPLING_RATE) {
        return;
    }

    auto ldrStatus = digitalRead(pin);

    this->lineDetected = (ldrStatus == LOW) ? true : false;
    this->lastUpdate = now;
}

void IRSensor::reset() {
    this->lineDetected = false;
    this->lastUpdate = 0;
}