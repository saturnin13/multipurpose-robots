#include "ButtonSensor.hpp"

#include <Arduino.h>

ButtonSensor::ButtonSensor(int pin) : pin(pin) {
    pinMode(pin, INPUT_PULLUP);
    this->sampling_rate = BUTTON_SAMPLING_RATE;
}

void ButtonSensor::update() {
    Sensor::update();

    unsigned long now = millis();

    int currentState = digitalRead(this->pin);


    if (this->lastDebounceTime != 0 && (now - this->lastDebounceTime) > DEBOUNCE_DELAY) {
        // The same button value has been held for longer than the debounce delay.
        this->pressed = (currentState == LOW) ? true : false;
        this->lastDebounceTime = 0;
    }

    this->previousButtonState = currentState;
}

void ButtonSensor::reset() {
    this->lastUpdate = 0;
}
