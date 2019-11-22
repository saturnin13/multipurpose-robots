#include "ButtonUpdateAgent.hpp"
#include <Arduino.h>


ButtonUpdateAgent::ButtonUpdateAgent(State* state, ButtonSensor* b): UpdateAgent(state), b(b) {

}

void ButtonUpdateAgent::update() {
    Serial.print(this->b->pressed);
    if(this->b->pressed && this->state->initTime == 0) {
        this->state->initTime = millis();
        this->state->robotState = ARMED;
    }

    //move this to own button, just quick fix for now
    if(this->b->pressed && this->state->robotState == ARMED) {
        this->state->emergencyStop = true;
    }

}