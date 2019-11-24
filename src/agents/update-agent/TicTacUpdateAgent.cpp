#include "TicTacUpdateAgent.hpp"

#define TOTAL_DROPPING_TIME 8000

TicTacUpdateAgent::TicTacUpdateAgent(State *state) : UpdateAgent(state) {}

void TicTacUpdateAgent::update() {

    auto now = millis();

    switch (this->state->ticTacState) {
    case DROPPED:
    case UNDROPPED:
        return;

    case REQUESTED:
        this->timeDropStart = now;
        this->state->ticTacState = DROPPING;
        break;

    case DROPPING:
        if (now - timeDropStart > TOTAL_DROPPING_TIME) {
            this->state->ticTacState = DROPPED;
        }
        break;
        
    }

}