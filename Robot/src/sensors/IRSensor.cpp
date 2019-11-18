#include "IRSensor.hpp"
#include <Arduino.h>

// The minimum number of milliseconds that should pass
// between each sample.
#define SAMPLE_FREQUENCY 100

IRSensor::IRSensor(int ledPin, int ldrPin) : ledPin(ledPin), ldrPin(ldrPin) {
    pinMode(ledPin, OUTPUT);
    pinMode(ldrPin, INPUT);
}

void IRSensor::update() {

    unsigned long now = millis();

    // Check if we can update the sensor data.
    if (now - lastUpdate < SAMPLE_FREQUENCY) {
        return;
    }

    auto ldrStatus = (unsigned int) analogRead(ldrPin);

    this->value = ldrStatus;
    this->lastUpdate = now;
}

void IRSensor::reset() {
    this->value = 0;
    this->lastUpdate = 0;
}