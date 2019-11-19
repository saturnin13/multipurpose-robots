#include "ButtonSensor.hpp"

#include <Arduino.h>

ButtonSensor::ButtonSensor(int pin) : pin(pin) {
    pinMode(pin, INPUT_PULLUP);
}

void ButtonSensor::update() {
    this->pressed = (digitalRead(this->pin) == HIGH) ? true : false;
    this->lastUpdate = millis();
}

void ButtonSensor::reset() {
    this->lastUpdate = 0;
}
