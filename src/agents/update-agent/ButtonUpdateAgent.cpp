#include "ButtonUpdateAgent.hpp"

ButtonUpdateAgent::ButtonUpdateAgent(State *state, ButtonSensor *button)
    : UpdateAgent(state), button(button) {}

void ButtonUpdateAgent::update() {

    if (this->button->pressed && this->state->robotState == DISARMED) {
        // armed so the led changes state
        Serial.println("ARMED");
        this->state->robotState = ARMED;
        return;
    }

    if (this->state->robotState == DISARMED &&
        this->state->finalTable != COMPLETED) {
        // TODO: make that the motors start now and not before -> bool moveRobot
        // = true;
    }

    // move this to own button or delete it for competition, but it is nice for
    // testing, just quick fix for now
    if (this->button->pressed && this->state->emergencyStop) {
        Serial.println("ESTOP RESET");
        this->state->emergencyStop = false;
    }
}