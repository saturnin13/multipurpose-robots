#include <Arduino.h>

#include "LineDetectionUpdateAgent.hpp"

#define PRINT_LINE_STATE 0


LineDetectionUpdateAgent::LineDetectionUpdateAgent(State* state, LineFollowerSensor* lf, UltrasonicSensor* usNW, UltrasonicSensor* usNE)
: UpdateAgent(state), lf(lf), usNW(usNW), usNE(usNE) {

}

void LineDetectionUpdateAgent::update() {

    //if we already completed this table, we do not have to look for lines anymore
    if(this->state->lineFollowingTable == COMPLETED) {
        return;
    }

    // Get sensor values
    bool lf0 = lf->lineDetected0;
    bool lf1 = lf->lineDetected1;
    bool lf2 = lf->lineDetected2;
    bool lf3 = lf->lineDetected3;
    bool lf4 = lf->lineDetected4;

    bool usNWIsEdge = usNW->distance > US_EDGE_THRESHOLD;
    bool usNEIsEdge = usNE->distance > US_EDGE_THRESHOLD;

    // Checking the current state of the table and updating if necessary
    if (!lf0 && !lf1 && !lf2 && !lf3 && !lf4 && state->lineFollowingTable == CURRENT) {
        state->lineFollowingTable = COMPLETED;

    } else if ((lf0 || lf1 || lf2 || lf3 || lf4) && !usNWIsEdge && !usNEIsEdge) {
        state->lineFollowingTable = CURRENT;

        // Updating the line state
        if (lf0) {
            state->lineState = LEFT;
            if(PRINT_LINE_STATE) {Serial.println("LEFT");}
        } else if (lf) {
            state->lineState = LEFT;
            if(PRINT_LINE_STATE) {Serial.println("LEFT");}
        } else if (lf3) {
            state->lineState = RIGHT;
            if(PRINT_LINE_STATE) {Serial.println("RIGHT");}
        } else if (lf4) {
            state->lineState = RIGHT;
            if(PRINT_LINE_STATE) {Serial.println("RIGHT");}
        } else if (lf2) {
            state->lineState = CENTER;
            if(PRINT_LINE_STATE) {Serial.println("CENTER");}
        } else {
            Serial.println("Unexpected state in LineDetectionUpdateAgent");
        }
    }
}