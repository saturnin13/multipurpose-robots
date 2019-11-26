#include "Motor.hpp"
#include <Arduino.h>

#define MIN_STOP_TIME 100

Motor::Motor(int in1Pin, int in2Pin, int enaPin)
    : in1Pin(in1Pin), in2Pin(in2Pin), enaPin(enaPin) {
    this->pwmSpeed = 0;
    this->forward = true;
    this->stopTime = millis();
    this->stopped = true;

    pinMode(in1Pin, OUTPUT);
    pinMode(in2Pin, OUTPUT);
    pinMode(enaPin, OUTPUT);
}

void Motor::configure(bool forward, int speed) {
    this->forward = forward;
    this->pwmSpeed = map(speed, 0, 100, 0, 255);
    this->stopped = false;
}

void Motor::enact() {

    unsigned long now = millis();
    // Check if we can update the motor state.
    if (now - stopTime < MIN_STOP_TIME) {
        return;
    }

    // Save if we are moving or not
    if (this->pwmSpeed == 0 && !this->stopped) {
        this->stopped = true;
        stopTime = millis();
    } else if (this->pwmSpeed != 0) {
        this->stopped = false;
    }

    if (this->forward) {
        // Forward
        digitalWrite(this->in1Pin, LOW);
        digitalWrite(this->in2Pin, HIGH);
        analogWrite(this->enaPin, pwmSpeed);
    } else {
        // Reverse
        digitalWrite(this->in1Pin, HIGH);
        digitalWrite(this->in2Pin, LOW);
        analogWrite(this->enaPin, pwmSpeed);
    }
}