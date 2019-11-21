#include "LEDActionAgent.hpp"


LEDActionAgent::LEDActionAgent(State* state, LED* led): state(state), led(led) {
    led->configure(SOLID_RED);
}

void LEDActionAgent::enact() {
    if ((this->state->robotState == DISARMED) && (this->state->finalTable != COMPLETED)) {
        this->led->configure(SOLID_RED);
    } else if(this->state->robotState == ARMED) {
        this->led->configure(STROBE_GREEN);
    } else if ((this->state->robotState == DISARMED) && (this->state->finalTable == COMPLETED)) {
        this->led->configure(FLASH_BLUE);
    }
    led->enact();
}