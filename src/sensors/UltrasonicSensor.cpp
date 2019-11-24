#include "UltrasonicSensor.hpp"
#include <Arduino.h>

// The minimum number of milliseconds that should pass
// between each sample.
#define SAMPLE_FREQUENCY 500

//  The maximum distance the sensor is expected to read.
#define MAX_DISTANCE 200

UltrasonicSensor::UltrasonicSensor(int triggerPin, int echoPin) : triggerPin(triggerPin), echoPin(echoPin), sonar(triggerPin, echoPin, MAX_DISTANCE) {
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

void UltrasonicSensor::update() {

    unsigned long now = millis();
    
    // Check if we can update the sensor data.
    if (now - lastUpdate < SAMPLE_FREQUENCY) {
        return;
    }

    unsigned int uS = sonar.ping();
    this->distance = sonar.convert_cm(uS);
    this->lastUpdate = millis();

}

void UltrasonicSensor::reset() {
    this->distance = 0;
    this->lastUpdate = 0;
}