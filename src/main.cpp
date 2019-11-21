#include <Arduino.h>

#include "sensors/ButtonSensor.hpp"
#include "actuators/LED.hpp"
#include "sensors/IMUSensor.hpp"
#include "sensors/IRSensor.hpp"
#include "sensors/LineFollowerSensor.hpp"
#include "sensors/UltrasonicSensor.hpp"

// Example of how to create ultrasonic sensors.
UltrasonicSensor usFrontLeft(46, 47);
UltrasonicSensor usFrontRight(44, 45);
UltrasonicSensor usFrontMiddle(42, 43);
UltrasonicSensor usDownLeft(50, 51);
UltrasonicSensor usDownRight(52, 53);
UltrasonicSensor usBackLeft(48, 49);

// Example of how to create IMU
IMUSensor imu;

// Example of how to create IR sensor
IRSensor ir1(13, 30);
IRSensor ir2(13, 31);
IRSensor ir3(13, 32);
IRSensor ir4(13, 33);

// Example of how to a button
ButtonSensor b1(12);

// Example of LED
LED l1(10, 9, 8);

// Example of how to create line follower sensor
int linefollowerPins[] = {A0, A1, A2, A3, A4};
LineFollowerSensor lf(linefollowerPins, 5);

int counter = 0;
long initTime;
long ledTime;

#include<Wire.h>

void setup() {
  
    Wire.endTransmission(true); //otherwise it doesnt work

    Serial.begin(9600);
    Serial.print(" Setup done ");
    l1.configure(SOLID_RED);
    initTime = millis();
    
}

void loop() {
    counter++;
    b1.update();
    usFrontLeft.update();
    usFrontMiddle.update();
    usFrontRight.update();
    usDownLeft.update();
    usDownRight.update();
    usBackLeft.update();
    //ir1.update();
    //ir2.update();
    //ir3.update();
    //ir4.update();
    //lf.update();
    //imu.update();
    l1.enact();

    if(b1.pressed) {
        l1.configure(STROBE_GREEN);
        ledTime = millis();
    }

    //Serial.print(usFrontLeft.distance);Serial.print(" , ");
    //Serial.print(usFrontMiddle.distance);Serial.print(" , ");
    //Serial.print(usFrontRight.distance);Serial.print(" , ");
    //Serial.print(usDownLeft.distance);Serial.print(" , ");
    //Serial.print(usDownRight.distance);Serial.print(" , ");
    //Serial.print(usBackLeft.distance);Serial.print(" - ");
    //Serial.print(ir1.value);Serial.print(" , ");
    //Serial.print(ir2.value);Serial.print(" , ");
    //Serial.print(ir3.value);Serial.print(" , ");
    //Serial.print(ir4.value);Serial.print(" , ");
    //Serial.print(lf.values[0]);Serial.print(" , ");
    //Serial.print(lf.values[1]);Serial.print(" , ");
    //Serial.print(lf.values[2]);Serial.print(" , ");
    //Serial.print(lf.values[3]);Serial.print(" , ");
    //Serial.print(lf.values[4]);Serial.print(" , ");
    //Serial.print(imu.xAngle);Serial.print(" , ");
    //Serial.print(imu.yAngle);Serial.print(" , ");
    //Serial.println(b1.pressed);
    //Serial.println(' ');
    if((millis() - ledTime)> 5000) {
        l1.configure(FLASH_BLUE);
    }

    // Update state_machine machine

    // Carry out actions
}