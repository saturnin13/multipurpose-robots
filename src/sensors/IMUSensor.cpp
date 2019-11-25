#include "IMUSensor.hpp"

#include <Arduino.h>
#include <Wire.h>

#define I2C_ADDRESS 0x68
#define POWER_MGMT_REGISTER 0x6B
#define START_REGISTER 0x3B
#define MIN_VAL 265
#define MAX_VAL 402

IMUSensor::IMUSensor() {
     
    Wire.begin();
    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(POWER_MGMT_REGISTER);
    Wire.write(0); // Wake up the IMU
    //Wire.endTransmission(true);

}

void IMUSensor::update() {
    Sensor::update();

    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(START_REGISTER);
    Wire.endTransmission(false);
    Wire.requestFrom(I2C_ADDRESS, 6, true); // Request contents of 6 registers

    int accX = Wire.read() << 8 | Wire.read();
    int accY = Wire.read() << 8 | Wire.read();
    int accZ = Wire.read() << 8 | Wire.read();
    // int temp = Wire.read() << 8 | Wire.read();
    //int gyroX = Wire.read() << 8 | Wire.read();
    //int gyroY = Wire.read() << 8 | Wire.read();
    //int gyroZ = Wire.read() << 8 | Wire.read();

    int xAng = map(accX, MIN_VAL, MAX_VAL, -90, 90);
    int yAng = map(accY, MIN_VAL, MAX_VAL, -90, 90);
    int zAng = map(accZ, MIN_VAL, MAX_VAL, -90, 90);

    this->xAngle = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
    this->yAngle = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
    this->zAngle = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);
}

void IMUSensor::reset() {
    this->xAngle = 0;
    this->yAngle = 0;
    this->zAngle = 0;
    this->lastUpdate = 0;
}
