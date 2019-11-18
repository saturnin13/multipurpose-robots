#include <Arduino.h>

#include "sensors/UltrasonicSensor.hpp"
#include "sensors/IMUSensor.hpp"
#include "sensors/IRSensor.hpp"

// Example of how to create two ultrasonic sensors.
UltrasonicSensor usFrontLeft(DD1, DD2);
UltrasonicSensor usFrontRight(DD3, DD4);

// Example of how to create IMU
IMUSensor imu;

// Example of how to create IR sensor
IRSensor ir1(13, A0);

void setup() {
    Serial.begin(9600);
}

void loop() {
    
    // Update sensors
    usFrontLeft.update();
    usFrontRight.update();
    imu.update();
    ir1.update();

    // Update state machine

    // Carry out actions

}