#include "LineFollowerSensor.hpp"
#include <Arduino.h>

// Sampling rate in milliseconds
#define SAMPLING_RATE 20

// Debouncing delay in number of samples
#define DEBOUNCE_COUNT 3

#define IR_LF_THRESHOLD 400

LineFollowerSensor::LineFollowerSensor(int pin0, int pin1, int pin2, int pin3, int pin4)
    : pin0(pin0), pin1(pin1), pin2(pin2), pin3(pin3), pin4(pin4) {
    
}

void LineFollowerSensor::update() {
    unsigned long now = millis();

    // Check if we can update the sensor data.
    if (now - this->lastUpdate < SAMPLING_RATE) {
        return;
    }

    // int pins[] = {pin0, pin1, pin2, pin3, pin4};
    // int pinCount = 5;

    // for (int i = 0; i < pinCount; i++) {

    // }

    // IR 0
    bool currentLine0 = analogRead(this->pin0) < IR_LF_THRESHOLD;

    if (currentLine0 != this->lineDetected0) {
        if (line0DetectionCount >= DEBOUNCE_COUNT) {
            this->lineDetected0 = currentLine0;
            this->line0DetectionCount = 0;
        } else {
            this->line0DetectionCount += 1;
        }
    }

    // IR 1
    bool currentLine1 = analogRead(this->pin1) < IR_LF_THRESHOLD;

    if (currentLine1 != this->lineDetected1) {
        if (line1DetectionCount >= DEBOUNCE_COUNT) {
            this->lineDetected1 = currentLine1;
            this->line1DetectionCount = 0;
        } else {
            this->line1DetectionCount += 1;
        }
    }

    // IR 2
    bool currentLine2 = analogRead(this->pin2) < IR_LF_THRESHOLD;

    if (currentLine2 != this->lineDetected2) {
        if (line2DetectionCount >= DEBOUNCE_COUNT) {
            this->lineDetected2 = currentLine2;
            this->line2DetectionCount = 0;
        } else {
            this->line2DetectionCount += 1;
        }
    }

    // IR 3
    bool currentLine3 = analogRead(this->pin3) < IR_LF_THRESHOLD;

    if (currentLine3 != this->lineDetected3) {
        if (line3DetectionCount >= DEBOUNCE_COUNT) {
            this->lineDetected3 = currentLine3;
            this->line3DetectionCount = 0;
        } else {
            this->line3DetectionCount += 1;
        }
    }

    // IR 4
    bool currentLine4 = analogRead(this->pin4) < IR_LF_THRESHOLD;

    if (currentLine4 != this->lineDetected4) {
        if (line4DetectionCount >= DEBOUNCE_COUNT) {
            this->lineDetected4 = currentLine4;
            this->line4DetectionCount = 0;
        } else {
            this->line4DetectionCount += 1;
        }
    }

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