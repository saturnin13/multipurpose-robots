#include "LEDActionAgent.hpp"
#include "ActionAgent.hpp"

LEDActionAgent::LEDActionAgent(State* state, LED* led): ActionAgent(state), led(led) {
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