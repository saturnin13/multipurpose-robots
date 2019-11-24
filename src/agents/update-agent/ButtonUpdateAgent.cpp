#include "ButtonUpdateAgent.hpp"
#include <Arduino.h>
#include "../../Constants.hpp"


ButtonUpdateAgent::ButtonUpdateAgent(State* state, ButtonSensor* b): UpdateAgent(state), b(b) {

}

void ButtonUpdateAgent::update() {

    if(this->b->pressed) {
        //armed so the led changes state
        this->state->robotState = ARMED;
    }

    //TODO in state agent
    unsigned long now = millis();
    if(now - this->state->initializationTime > START_DELAY_TIME && this->state->robotState == DISARMED && this->state->finalTable != COMPLETED) {
        //TODO: make that the motors start now and not before -> bool moveRobot = true;
    }

    //move this to own button or delete it for competition, but it is nice for testing, just quick fix for now
    if(this->b->pressed && this->state->emergencyStop) {
        this->state->emergencyStop = false;
    }

}