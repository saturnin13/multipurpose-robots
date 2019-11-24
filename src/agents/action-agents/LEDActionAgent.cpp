#include "LEDActionAgent.hpp"
#include "ActionAgent.hpp"
#include <Arduino.h>

LEDActionAgent::LEDActionAgent(State* state, LED* led): ActionAgent(state), led(led) {
   //led->configure(SOLID_RED);
   //led->enact();
}

void LEDActionAgent::enact() {
    if ((this->state->robotState == DISARMED) && (this->state->finalTable != COMPLETED)) {
        this->led->configure(SOLID_RED);
        //Serial.println("RED");
    } else if(this->state->robotState == ARMED) {
        this->led->configure(STROBE_GREEN);
        //Serial.println("GREEN");

    } else if ((this->state->robotState == DISARMED) && (this->state->finalTable == COMPLETED)) {
        this->led->configure(FLASH_BLUE);
        //Serial.println("BLUE");

    }

    led->enact();
}