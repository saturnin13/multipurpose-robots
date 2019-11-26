#include "LEDActionAgent.hpp"
#include "ActionAgent.hpp"
#include <Arduino.h>

LEDActionAgent::LEDActionAgent(State *state, LED *led)
    : ActionAgent(state), led(led) {}

void LEDActionAgent::enact() {

    if ((this->state->robotState == DISARMED) &&
        (this->state->finalTable != COMPLETED)) {
        this->led->configure(SOLID_RED);
        //Serial.println("RED");

    } else if (this->state->robotState == ARMED) {
        this->led->configure(STROBE_GREEN);
        //Serial.println("GREEN");

    } else if ((this->state->robotState == DISARMED) &&
               (this->state->finalTable == COMPLETED)) {
        this->led->configure(FLASH_BLUE);
        //Serial.println("BLUE");
    } else {
        Serial.println("NO LED STATE SELECTED; THIS SHOULD NEVER HAPPEN; WE HAVE A SERIOUS PROBLEM: FIX IT NOW");
    }

    led->enact();
}