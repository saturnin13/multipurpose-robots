#include "UltrasonicSensor.hpp"
#include <Arduino.h>

// The minimum number of milliseconds that should pass
// between each sample.
#define SAMPLE_FREQUENCY 250

// The speed of sound in millimeters per microsecond.
#define SPEED_OF_SOUND 0.34027

UltrasonicSensor::UltrasonicSensor(int triggerPin, int echoPin) : triggerPin(triggerPin), echoPin(echoPin) {
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

void UltrasonicSensor::update() {

    unsigned long now = millis();
    
    // Check if we can update the sensor data.
    if (now - lastUpdate < SAMPLE_FREQUENCY) {
        return;
    }

    // TODO: Should we refactor this so it doesn't use delays?
    //       I'm not sure we can get microsecond-level time precision.

    // Send pulse
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);

    // Read time of flight (in microseconds)
    long duration = pulseIn(echoPin, HIGH);    

    this->distance = microsecondsToMillimeters(duration);
    this->lastUpdate = millis();

}

void UltrasonicSensor::reset() {
    this->distance = 0;
    this->lastUpdate = 0;
}

long UltrasonicSensor::microsecondsToMillimeters(long time) {
    // We divide by 2 since we are measuring the distance
    // to the nearest obstacle and not the total
    // distance that the sound travels.
    return round(time / SPEED_OF_SOUND / 2);
}