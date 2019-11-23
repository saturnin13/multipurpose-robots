#include "LineFollowerSensor.hpp"
#include <Arduino.h>

#define IR_THRESHOLD 500

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

    this->value0 = (unsigned int)analogRead(this->pin0);
    this->value1 = (unsigned int)analogRead(this->pin1);
    this->value2 = (unsigned int)analogRead(this->pin2);
    this->value3 = (unsigned int)analogRead(this->pin3);
    this->value4 = (unsigned int)analogRead(this->pin4);

}

void LineFollowerSensor::reset() {
    this->value0 = 0;
    this->value1 = 0;
    this->value2 = 0;
    this->value3 = 0;
    this->value4 = 0;
    this->lastUpdate = 0;
}

bool LineFollowerSensor::unanimousDetection() {
    return (value0 > IR_THRESHOLD) &&
           (value1 > IR_THRESHOLD) &&
           (value2 > IR_THRESHOLD) &&
           (value3 > IR_THRESHOLD) &&
           (value4 > IR_THRESHOLD);
}