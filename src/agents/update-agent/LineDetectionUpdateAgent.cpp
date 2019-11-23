#include <Arduino.h>

#include "LineDetectionUpdateAgent.hpp"


LineDetectionUpdateAgent::LineDetectionUpdateAgent(State* state, LineFollowerSensor* lf): UpdateAgent(state), lf(lf) {

}

void LineDetectionUpdateAgent::update() {
    // Get sensor values
    bool lf0 = lf->lineDetected0;
    bool lf1 = lf->lineDetected1;
    bool lf2 = lf->lineDetected2;
    bool lf3 = lf->lineDetected3;
    bool lf4 = lf->lineDetected4;

    // Checking for edges to avoid erronous line detections
    if(state->northEntity == EDGE) {
        state->lineState = LOST;
        return;
    }

    // Checking the current state of the table and updating if necessary
    if (!lf0 && !lf1 && !lf2 && !lf3 && !lf4 && state->lineFollowingTable == CURRENT) {
        state->lineFollowingTable = COMPLETED;
    } else if (lf0 && lf1 && lf2 && lf3 && lf4) {
        state->lineFollowingTable = CURRENT;
    }
    
    // Updating the line state
     if (lf0 && !lf1 && !lf2 && !lf3 && !lf4) {
        state->lineState = LEFT;
    } else if (lf0) {
        state->lineState = LEFT;
    } else if (lf1 && !lf2) {
        state->lineState = LEFT;
    } else if (lf2) {
        state->lineState = CENTER;
    } else if (lf3 && !lf2) {
        state->lineState = RIGHT;
    } else if (lf4) {
        state->lineState = RIGHT;
    } else {
        state->lineState = LOST;
    }
}