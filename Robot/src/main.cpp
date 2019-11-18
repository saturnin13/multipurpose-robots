#include <Arduino.h>

#include "sensors/UltrasonicSensor.hpp"
#include "sensors/IMUSensor.hpp"

// Example of how to create two ultrasonic sensors.
UltrasonicSensor usFrontLeft(DD1, DD2);
UltrasonicSensor usFrontRight(DD3, DD4);

// Example of how to create IMU
IMUSensor imu;

void setup() {
    Serial.begin(9600);
}

void loop() {
    
    // Update sensors
    usFrontLeft.update();
    usFrontRight.update();
    imu.update();

    // Update state machine

    // Carry out actions

}