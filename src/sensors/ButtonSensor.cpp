#include "ButtonSensor.hpp"

#include <Arduino.h>

ButtonSensor::ButtonSensor(int pin) : pin(pin) {
    pinMode(pin, INPUT_PULLUP);
}

void ButtonSensor::update() {

    unsigned long now = millis();
    int currentState = digitalRead(this->pin);

    if (currentState != this->previousButtonState) {
        // Reset the debouncing timer.
        lastDebounceTime = now;
    }

    if (this->lastDebounceTime != 0 && (now - this->lastDebounceTime) > DEBOUNCE_DELAY) {
        // The same button value has been held for longer than the debounce delay.
        this->pressed = (currentState == LOW) ? true : false;
        this->lastDebounceTime = 0;
    }

    this->previousButtonState = currentState;
    this->lastUpdate = now;

}

void ButtonSensor::reset() {
    this->lastUpdate = 0;
}
