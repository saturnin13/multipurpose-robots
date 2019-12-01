#include "TicTacUpdateAgent.hpp"

#define TOTAL_DROPPING_TIME 8000

TicTacUpdateAgent::TicTacUpdateAgent(State *state) : UpdateAgent(state) {
    this->timeDropStart = 0;
}

void TicTacUpdateAgent::update() {
    if(DEBUG && TICTAC_UPDATE_AGENT_DEBUG){Serial.print("\nTicTacUpdateAgent: ");}
    if(!(this->state->ticTacState == CURRENT) && this->state->incline == COMPLETED) {
        if(DEBUG && TICTAC_UPDATE_AGENT_DEBUG){Serial.println("ACTIVATING THE TICTAC DROPPING");}
        this->state->ticTacState = CURRENT;
        this->timeDropStart = millis();
    }

    switch (this->state->ticTacState) {
    case COMPLETED:
    case UNSEEN:
        return;

    case CURRENT:
        if(DEBUG && TICTAC_UPDATE_AGENT_DEBUG){Serial.println("IN DROPPING STATE");}
unsigned long now = millis();

        if (now - timeDropStart > TOTAL_DROPPING_TIME) {
            this->state->ticTacState = COMPLETED;
        }
        break;

    }
}