#include "UltrasonicSensor.hpp"
#include <Arduino.h>

UltrasonicSensor::UltrasonicSensor(int triggerPin, int echoPin) : triggerPin(triggerPin), echoPin(echoPin), sonar(triggerPin, echoPin, MAX_DISTANCE) {
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);
    this->sampling_frequency = ULTRASONIC_SAMPLING_FREQUENCY;
}

void UltrasonicSensor::update() {
    Sensor::update();

    unsigned int uS = sonar.ping();
    this->distance = sonar.convert_cm(uS);
}

void UltrasonicSensor::reset() {
    this->distance = 0;
    this->lastUpdate = 0;
}