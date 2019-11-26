#include <Arduino.h>

#include "LineDetectionUpdateAgent.hpp"


LineDetectionUpdateAgent::LineDetectionUpdateAgent(State* state, LineFollowerSensor* lf, UltrasonicSensor* usNW, UltrasonicSensor* usNE)
: UpdateAgent(state), lf(lf), usNW(usNW), usNE(usNE) {

}

void LineDetectionUpdateAgent::update() {
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
        if (lf2) {
            state->lineState = CENTER;
            //Serial.println("CENTER");
        } else if (lf0) { //TODO lf1 and lf2 should be left as well: && !lf2
            state->lineState = LEFT;
            //Serial.println("LEFT");
        } else if (lf1) {
            state->lineState = LEFT;
            //Serial.println("LEFT");
        } else if (lf3) { // && !lf2
            state->lineState = RIGHT;
            //Serial.println("RIGHT");
        } else if (lf4) {
            state->lineState = RIGHT;
            //Serial.println("RIGHT");
        } else {
            Serial.println("Unexpected state in LineDetectionUpdateAgent");
        }
    }
}