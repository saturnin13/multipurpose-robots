#include "TicTacUpdateAgent.hpp"

#define TOTAL_DROPPING_TIME 8000



TicTacUpdateAgent::TicTacUpdateAgent(State* state, ros::NodeHandle* nh, ros::Subscriber<std_msgs::Empty> subTT): UpdateAgent(state), nh(nh), subTT(subTT) {

    this->nh->subscribe(this->subTT);

    this->dropRequested = false;
    this->timeDropStart = 0;
}

void TicTacUpdateAgent::update() {
    update(false);
}


void TicTacUpdateAgent::update(bool dropRequested) {
    if(!this->dropRequested && dropRequested) {
        this->dropRequested = dropRequested;
    }

    if(this->state->ticTacState == DROPPED || this->state->ticTacState == UNDROPPED) {
        return;
    }

    if(!this->state->ticTacState == DROPPING && dropRequested) {
        timeDropStart = millis();
        state->ticTacState = DROPPING;
    } else if (this->state->ticTacState == DROPPING && millis() - timeDropStart > TOTAL_DROPPING_TIME) {
        this->state->ticTacState == DROPPED;
    }
}