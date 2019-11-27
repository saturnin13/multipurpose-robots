#include "TicTacUpdateAgent.hpp"

#define TOTAL_DROPPING_TIME 8000

TicTacUpdateAgent::TicTacUpdateAgent(State *state) : UpdateAgent(state) {
    this->timeDropStart = 0;
}

void TicTacUpdateAgent::update() {

    if(!(this->state->ticTacState == CURRENT) && this->state->incline == COMPLETED) {
        this->state->ticTacState = CURRENT;
        this->timeDropStart = millis();
    }

    switch (this->state->ticTacState) {
    case COMPLETED:
    case UNSEEN:
        return;

    case CURRENT:
        unsigned long now = millis();

        if (now - timeDropStart > TOTAL_DROPPING_TIME) {
            this->state->ticTacState = COMPLETED;
        }
        break;

    }
}