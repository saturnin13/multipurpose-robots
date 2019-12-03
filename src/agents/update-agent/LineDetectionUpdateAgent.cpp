#include <Arduino.h>

#include "LineDetectionUpdateAgent.hpp"

LineDetectionUpdateAgent::LineDetectionUpdateAgent(State* state, LineFollowerSensor* lf, UltrasonicSensor* usNW, UltrasonicSensor* usNE)
: UpdateAgent(state), lf(lf), usNW(usNW), usNE(usNE) {

}

void LineDetectionUpdateAgent::update() {
    if (DEBUG && LINE_DETECTION_UPDATE_AGENT_DEBUG){Serial.print("\nLineDetectionUpdateAgent: ");}
    
    // if we already completed this table, we do not have to look for lines anymore
    if (this->state->lineFollowingTable == COMPLETED) {
        if(DEBUG && LINE_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("LINE FOLLOWING COMPLETED, RETURNING");}
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
        if(DEBUG && LINE_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("SETTING LINE FOLLOWING TO COMPLETED");}
        state->lineFollowingTable = COMPLETED;
    
    } else if ((lf0 || lf1 || lf2 || lf3 || lf4) && !usNWIsEdge && !usNEIsEdge) {
        state->lineFollowingTable = CURRENT;
        if(DEBUG && LINE_DETECTION_UPDATE_AGENT_DEBUG){Serial.print("Following the Line to the ");}

        // Updating the line state
        if (lf0) {
            if(DEBUG && LINE_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("LINE LEFT");}
            state->lineState = LEFT;
        } else if (lf1) {
            if(DEBUG && LINE_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("LINE LEFT*");}
            state->lineState = LEFT;
        } else if (lf3) {
            if(DEBUG && LINE_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("LINE RIGHT");}
            state->lineState = RIGHT;
        } else if (lf4) {
            if(DEBUG && LINE_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("LINE RIGHT*");}
            state->lineState = RIGHT;
        } else if (lf2) {
            if(DEBUG && LINE_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("LINE CENTER");}
            state->lineState = CENTER;
        }
    }
}