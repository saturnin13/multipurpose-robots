#include "LEDActionAgent.hpp"
#include "ActionAgent.hpp"
#include <Arduino.h>

LEDActionAgent::LEDActionAgent(State *state, LED *led)
    : ActionAgent(state), led(led) {}

void LEDActionAgent::enact() {
    if(DEBUG && LED_ACTION_AGENT_DEBUG){Serial.print("\nLEDActionAgent: ");}

    if ((this->state->robotState == DISARMED) &&
        (this->state->finalTable != COMPLETED)) {
        if(DEBUG && LED_ACTION_AGENT_DEBUG){Serial.println("RED");}
        this->led->configure(SOLID_RED);

    } else if (this->state->robotState == ARMED) {
        if(DEBUG && LED_ACTION_AGENT_DEBUG){Serial.println("GREEN");}
        this->led->configure(STROBE_GREEN);

    } else if ((this->state->robotState == DISARMED) &&
               (this->state->finalTable == COMPLETED)) {
        if(DEBUG && LED_ACTION_AGENT_DEBUG){Serial.println("BLUE");}
        this->led->configure(FLASH_BLUE);

    } else {
        Serial.println("NO LED STATE SELECTED; THIS SHOULD NEVER HAPPEN; WE HAVE A SERIOUS PROBLEM: FIX IT NOW");
    }

    led->enact();
}