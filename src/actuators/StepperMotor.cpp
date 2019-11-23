#include "StepperMotor.hpp"
#include <Arduino.h>


StepperMotor::StepperMotor(int in1Pin, int in2Pin, int in3Pin, int in4Pin)
: in1Pin(in1Pin), in2Pin(in2Pin), in3Pin(in3Pin), in4Pin(in4Pin) {
    this->stepNumber = 0;
    this->numberSteps = 0; //0 means always
    this->workInProgress = false;
    
    pinMode(in1Pin, OUTPUT);
    pinMode(in2Pin, OUTPUT);
    pinMode(in3Pin, OUTPUT);
    pinMode(in4Pin, OUTPUT);
}

void StepperMotor::configure(bool clockwise, int stepTime, int numberSteps) {
    this->clockwise = clockwise;
    this->stepTime = stepTime;
    this->numberSteps = numberSteps;
    this->workInProgress = true;
}

void StepperMotor::enact() {
    
    unsigned long now = millis();
    // Check if we can update the motor state.
    if (now - lastUpdate < this->stepTime) {
        return;
    }

    //check if we should continue doing steps
    if(this->numberSteps <= 0) {
        this->workInProgress = false;
        return;
    } else if (this->numberSteps > 0){
        this->numberSteps--;
    }

    //shoule be 0,1,2..7 for clockwise and 7,6,...0 for counterclockwise
    int mode = this->clockwise ? this->stepNumber : (7 - this->stepNumber);

    switch(mode){
        case 0:
        digitalWrite(this->in1Pin, LOW);
        digitalWrite(this->in2Pin, LOW);
        digitalWrite(this->in3Pin, LOW);
        digitalWrite(this->in4Pin, HIGH);
        break;
        case 1:
        digitalWrite(this->in1Pin, LOW);
        digitalWrite(this->in2Pin, LOW);
        digitalWrite(this->in3Pin, HIGH);
        digitalWrite(this->in4Pin, HIGH);
        break;
        case 2:
        digitalWrite(this->in1Pin, LOW);
        digitalWrite(this->in2Pin, LOW);
        digitalWrite(this->in3Pin, HIGH);
        digitalWrite(this->in4Pin, LOW);
        break;
        case 3:
        digitalWrite(this->in1Pin, LOW);
        digitalWrite(this->in2Pin, HIGH);
        digitalWrite(this->in3Pin, HIGH);
        digitalWrite(this->in4Pin, LOW);
        case 4:
        digitalWrite(this->in1Pin, LOW);
        digitalWrite(this->in2Pin, HIGH);
        digitalWrite(this->in3Pin, LOW);
        digitalWrite(this->in4Pin, LOW);
        break;
        case 5:
        digitalWrite(this->in1Pin, HIGH);
        digitalWrite(this->in2Pin, HIGH);
        digitalWrite(this->in3Pin, LOW);
        digitalWrite(this->in4Pin, LOW);
        break;
        case 6:
        digitalWrite(this->in1Pin, HIGH);
        digitalWrite(this->in2Pin, LOW);
        digitalWrite(this->in3Pin, LOW);
        digitalWrite(this->in4Pin, LOW);
        break;
        case 7:
        digitalWrite(this->in1Pin, HIGH);
        digitalWrite(this->in2Pin, LOW);
        digitalWrite(this->in3Pin, LOW);
        digitalWrite(this->in4Pin, HIGH);
    }

    
    //increase stepNumber
    stepNumber++;
    //reset if at 7
    stepNumber %= 8;

    this->lastUpdate = now;

}