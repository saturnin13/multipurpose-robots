#include "LineFollowerSensor.hpp"
#include <Arduino.h>

// The minimum number of milliseconds that should pass
// between each sample.
#define SAMPLE_FREQUENCY 100

LineFollowerSensor::LineFollowerSensor(int pin0, int pin1, int pin2, int pin3, int pin4)
    : pin0(pin0), pin1(pin1), pin2(pin2), pin3(pin3), pin4(pin4) {
    
}

void LineFollowerSensor::update() {

    unsigned long now = millis();

    // Check if we can update the sensor data.
    if (now - lastUpdate < SAMPLE_FREQUENCY) {
        return;
    }
    
    this->lineDetected0 = ((unsigned int)analogRead(this->pin0)) < IR_THRESHOLD;
    this->lineDetected1 = ((unsigned int)analogRead(this->pin1)) < IR_THRESHOLD;
    this->lineDetected2 = ((unsigned int)analogRead(this->pin2)) < IR_THRESHOLD;
    this->lineDetected3 = ((unsigned int)analogRead(this->pin3)) < IR_THRESHOLD;
    this->lineDetected4 = ((unsigned int)analogRead(this->pin4)) < IR_THRESHOLD;

}

void LineFollowerSensor::reset() {
    this->lineDetected0 = 0;
    this->lineDetected1 = 0;
    this->lineDetected2 = 0;
    this->lineDetected3 = 0;
    this->lineDetected4 = 0;
    this->lastUpdate = 0;
}

bool LineFollowerSensor::unanimousDetection() {
    return (lineDetected0 > IR_THRESHOLD) &&
           (lineDetected1 > IR_THRESHOLD) &&
           (lineDetected2 > IR_THRESHOLD) &&
           (lineDetected3 > IR_THRESHOLD) &&
           (lineDetected4 > IR_THRESHOLD);
}