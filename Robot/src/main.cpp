#include <Arduino.h>

#include "sensors/UltrasonicSensor.hpp"

// Example of how to create two ultrasonic sensors.
UltrasonicSensor usFrontLeft(DD1, DD2);
UltrasonicSensor usFrontRight(DD3, DD4);

void setup() {
    Serial.begin(9600);
}

void loop() {
    
    // Update sensors
    usFrontLeft.update();
    usFrontRight.update();

    // Update state machine

    // Carry out actions

}