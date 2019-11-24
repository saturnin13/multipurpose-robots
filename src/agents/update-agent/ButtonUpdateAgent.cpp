#include "ButtonUpdateAgent.hpp"
#include <Arduino.h>
#include "../../Constants.hpp"


ButtonUpdateAgent::ButtonUpdateAgent(State* state, ButtonSensor* b): UpdateAgent(state), b(b) {

}

void ButtonUpdateAgent::update() {
    //TODO in state agent
    unsigned long now = millis();
    //TODO && this->state->robotState == DISARMED
    if(this->b->pressed) {
        //armed so the led changes state
        Serial.println("ARMED");
        this->state->robotState = ARMED;
        return;
    }



    if(now - this->state->initializationTime > START_DELAY_TIME && this->state->robotState == DISARMED && this->state->finalTable != COMPLETED) {
        //TODO: make that the motors start now and not before -> bool moveRobot = true;
    }

    //move this to own button or delete it for competition, but it is nice for testing, just quick fix for now
    if(this->b->pressed && this->state->robotState == ARMED &&!this->state->emergencyStop) {
        Serial.println("ESTOP");
        this->state->emergencyStop = true;
    }

}