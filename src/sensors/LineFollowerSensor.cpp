#include "LineFollowerSensor.hpp"
#include <Arduino.h>

LineFollowerSensor::LineFollowerSensor(int pin0, int pin1, int pin2, int pin3, int pin4)
    : pin0(pin0), pin1(pin1), pin2(pin2), pin3(pin3), pin4(pin4) {
    
}

void LineFollowerSensor::update() {
    Sensor::update();

    this->lineDetected0 = analogRead(this->pin0) < IR_THRESHOLD;
    this->lineDetected1 = analogRead(this->pin1) < IR_THRESHOLD;
    this->lineDetected2 = analogRead(this->pin2) < IR_THRESHOLD;
    this->lineDetected3 = analogRead(this->pin3) < IR_THRESHOLD;
    this->lineDetected4 = analogRead(this->pin4) < IR_THRESHOLD;
}

void LineFollowerSensor::reset() {
    this->lineDetected0 = false;
    this->lineDetected1 = false;
    this->lineDetected2 = false;
    this->lineDetected3 = false;
    this->lineDetected4 = false;
    this->lastUpdate = 0;
}

bool LineFollowerSensor::unanimousDetection() {
    return lineDetected0 &&
           lineDetected1 &&
           lineDetected2 &&
           lineDetected3 &&
           lineDetected4;
}