#include "UltrasonicSensor.hpp"
#include <Arduino.h>

#define SAMPLING_RATE 200

//  The maximum distance the sensor is expected to read.
#define MAX_DISTANCE 150

UltrasonicSensor::UltrasonicSensor(int triggerPin, int echoPin) : triggerPin(triggerPin), echoPin(echoPin), sonar(triggerPin, echoPin, MAX_DISTANCE) {
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

void UltrasonicSensor::update() {
    unsigned long now = millis();

    // Check if we can update the sensor data.
    if (now - this->lastUpdate < SAMPLING_RATE) {
        return;
    }

    unsigned int uS = sonar.ping();
    unsigned int reading = sonar.convert_cm(uS);

    // Sometimes the sensor reads an incorrect value.
    // The value should never exceed MAX_DISTANCE.
    reading = max(reading, MAX_DISTANCE);

    // Account for the fact that the library reads
    // a distance of 0 if there is nothing within MAX_DISTANCE
    // of the sensor.
    if (reading == 0) {
        reading = MAX_DISTANCE;
    }

    this->distance = reading;
    this->lastUpdate = now;
}

void UltrasonicSensor::reset() {
    this->distance = 0;
    this->lastUpdate = 0;
}