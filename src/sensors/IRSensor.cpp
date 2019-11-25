#include "IRSensor.hpp"
#include <Arduino.h>

IRSensor::IRSensor(int pin) : pin(pin) {
    pinMode(pin, INPUT);
}

void IRSensor::update() {
    Sensor::update();

    auto ldrStatus = digitalRead(pin);

    this->lineDetected = (ldrStatus == LOW) ? true : false;
}

void IRSensor::reset() {
    this->lineDetected = false;
    this->lastUpdate = 0;
}