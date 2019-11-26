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


    //unsigned int now = millis();
    unsigned int uS = sonar.ping();
    //auto then = millis();
    //Serial.print("TIME FOR SENSORS: ");Serial.println(then-now);
    this->distance = sonar.convert_cm(uS);
    this->lastUpdate = now;
}

void UltrasonicSensor::reset() {
    this->distance = 0;
    this->lastUpdate = 0;
}