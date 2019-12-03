#include "ButtonSensor.hpp"
#include <Arduino.h>

// Sampling rate in milliseconds
#define SAMPLING_RATE 0

// Debouncing delay in milliseconds
#define DEBOUNCE_DELAY 50

ButtonSensor::ButtonSensor(int pin) : pin(pin) {
    pinMode(pin, INPUT_PULLUP);
    this->lastDebounceTime = 0;
    this->previousButtonState = -1;
}

void ButtonSensor::update() {

    unsigned long now = millis();

    // Check if we can update the sensor data.
    if (now - this->lastUpdate < SAMPLING_RATE) {
        return;
    }

    int currentState = digitalRead(this->pin);

    if ((now - this->lastDebounceTime) > DEBOUNCE_DELAY) {
        // The same button value has been held for longer than the debounce delay.
        this->pressed = (currentState == LOW) ? true : false;
        this->lastDebounceTime = millis();
    }

    this->previousButtonState = currentState;
    this->lastUpdate = now;
}

void ButtonSensor::reset() {
    this->lastUpdate = 0;
}
