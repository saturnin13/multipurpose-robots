#include "ButtonUpdateAgent.hpp"
#include "../../Constants.hpp"

ButtonUpdateAgent::ButtonUpdateAgent(State *state, ButtonSensor *button)
    : UpdateAgent(state), button(button) {
        this->lastButtonState = false;
        this->timeSinceArming = 0;
    }
void ButtonUpdateAgent::update() {
    if(DEBUG && BUTTON_UP_UPDATE_AGENT_DEBUG){Serial.print("\nButtonUpdateAgent: ");}
if (this->button->pressed && this->state->robotState == DISARMED) {
        // armed so the led changes state
        if(DEBUG && BUTTON_UP_UPDATE_AGENT_DEBUG){Serial.println("ARMED");}
this->state->robotState = ARMED;
        this->timeSinceArming=millis();
        this->lastButtonState = this->button->pressed;
        return;
    }

    if (this->state->robotState == ARMED && !this->state->move && (millis()-this->timeSinceArming)> NOT_MOVING_DELAY_AFTER_START) {
        // make that the motors start after 3 sec and not before
        if(DEBUG && BUTTON_UP_UPDATE_AGENT_DEBUG){Serial.println("ENABLING MOVEMENT");}
this->state->move = true;
    }

    //TODO move this to own button or delete it for competition, but it is nice for testing, just quick fix for now
    if(this->button->pressed && !this->lastButtonState && !this->state->emergencyStop) {
        if(DEBUG && BUTTON_UP_UPDATE_AGENT_DEBUG){Serial.println("ESTOP ACTIVATION");}
this->state->emergencyStop = true;
        this->lastButtonState = this->button->pressed;
        return;
    }

    if(this->button->pressed && !this->lastButtonState && this->state->emergencyStop) {
        if(DEBUG && BUTTON_UP_UPDATE_AGENT_DEBUG){Serial.println("ESTOP RESET");}
this->state->emergencyStop = false;
        this->lastButtonState = this->button->pressed;
        return;
    }

    

}