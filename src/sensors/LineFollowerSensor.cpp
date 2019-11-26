#include "LineFollowerSensor.hpp"
#include <Arduino.h>

// Sampling rate in milliseconds
#define SAMPLING_RATE 150

#define IR_LF_THRESHOLD 750

LineFollowerSensor::LineFollowerSensor(int pin0, int pin1, int pin2, int pin3, int pin4)
    : pin0(pin0), pin1(pin1), pin2(pin2), pin3(pin3), pin4(pin4) {
    
}

void LineFollowerSensor::update() {
    unsigned long now = millis();

    // Check if we can update the sensor data.
    if (now - this->lastUpdate < SAMPLING_RATE) {
        return;
    }

    this->lineDetected0 = analogRead(this->pin0) < IR_LF_THRESHOLD;
    this->lineDetected1 = analogRead(this->pin1) < IR_LF_THRESHOLD;
    this->lineDetected2 = analogRead(this->pin2) < IR_LF_THRESHOLD;
    this->lineDetected3 = analogRead(this->pin3) < IR_LF_THRESHOLD;
    this->lineDetected4 = analogRead(this->pin4) < IR_LF_THRESHOLD;
    this->lastUpdate = now;
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