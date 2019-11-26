#include "LEDActionAgent.hpp"
#include "ActionAgent.hpp"
#include <Arduino.h>

LEDActionAgent::LEDActionAgent(State *state, LED *led)
    : ActionAgent(state), led(led) {}

void LEDActionAgent::enact() {
    if ((this->state->robotState == DISARMED) &&
        (this->state->finalTable != COMPLETED)) {
        this->led->configure(SOLID_RED);

    } else if (this->state->robotState == ARMED) {
        this->led->configure(STROBE_GREEN);

    } else if ((this->state->robotState == DISARMED) &&
               (this->state->finalTable == COMPLETED)) {
        this->led->configure(FLASH_BLUE);
    }

    led->enact();
}