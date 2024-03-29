#include <Arduino.h>

#include "LineDetectionUpdateAgent.hpp"

LineDetectionUpdateAgent::LineDetectionUpdateAgent(State* state, LineFollowerSensor* lf, UltrasonicSensor* usNW, UltrasonicSensor* usNE)
: UpdateAgent(state), lf(lf), usNW(usNW), usNE(usNE) {
    this->hasCrossedLine = false;
    this->firstSeenSide = -1;
    this->lastSeenLineTime = 0;
}

void LineDetectionUpdateAgent::update() {
    if (DEBUG && LINE_DETECTION_UPDATE_AGENT_DEBUG){Serial.print("\nLineDetectionUpdateAgent: ");}

    // Get sensor values
    bool lf0 = lf->lineDetected0;
    bool lf1 = lf->lineDetected1;
    bool lf2 = lf->lineDetected2;
    bool lf3 = lf->lineDetected3;
    bool lf4 = lf->lineDetected4;

    bool usNWIsEdge = usNW->distance > US_EDGE_THRESHOLD;
    bool usNEIsEdge = usNE->distance > US_EDGE_THRESHOLD;

    // if we already completed this table, we do not have to look for lines anymore
    if (this->state->lineFollowingTable == COMPLETED) {
        if(DEBUG && LINE_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("LINE FOLLOWING COMPLETED, RETURNING");}
        return;
    }

    if (this->state->lineFollowingTable == UNSEEN && !lf0 && !lf1) {
        if(DEBUG && LINE_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("NO EXTERNAL SENSOR ACTIVATED, RETURNING");}
        return;
    }

    //Serial.print(lf0);
    //Serial.print(lf1);
    //Serial.print(lf2);
    //Serial.print(lf3);
    //Serial.print(lf4);

    signed int sumSensors = 0;

    // Checking the current state of the table and updating if necessary
    if (!lf0 && !lf1 && !lf2 && !lf3 && !lf4) {
        if(state->lineFollowingTable == CURRENT && millis() - this->lastSeenLineTime > LINE_FOLLOWING_COMPLETED_DELAY) {
            if (DEBUG && LINE_DETECTION_UPDATE_AGENT_DEBUG) {Serial.println("SETTING LINE FOLLOWING TO COMPLETED");}
            state->lineFollowingTable = COMPLETED;
        }

    } else {
        if(!usNWIsEdge && !usNEIsEdge) {
            if(DEBUG && LINE_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("FOLLOWING THE LINE");}
            this->lastSeenLineTime = millis();
            state->lineFollowingTable = CURRENT;

            // Evaluating if the first pin to see the line is the left most or right most
            if(this->firstSeenSide == -1) {
                this->firstSeenSide = lf3 || lf4 ? 4: 0;
            }

            // For case when directly facing the line
            if(lf0 && lf1 && lf2 && lf3 && lf4) {
                if(DEBUG && LINE_DETECTION_UPDATE_AGENT_DEBUG){Serial.print("COMPLETLY FACING THE LINE");}
                state->lineState = LEFT;

            // Case for before the robot line following sensor crossed the line
            } else if (lf0 && lf1 && !lf2 && lf3 && lf4) {
                if(DEBUG && LINE_DETECTION_UPDATE_AGENT_DEBUG){Serial.print("Special case of the line intersection");}
                state->lineState = CENTER;

            } else if (!this->hasCrossedLine) {
                if(DEBUG && LINE_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("NOT YET CROSSED THE LINE");}

                state->lineState = CENTER;
                if((!lf0 && this->firstSeenSide == 0) || (!lf4 && this->firstSeenSide == 4)) {
                    this->hasCrossedLine = true;
                }

            // Case for the intersection of lines
            } else if ((lf0 && lf1 && lf2 && lf3) || (lf1 && lf2 && lf3 && lf4)) {
                if(DEBUG && LINE_DETECTION_UPDATE_AGENT_DEBUG){Serial.print("Special case of the line intersection");}
                state->lineState = lf0 ? RIGHT: LEFT;
            }

            // Normal line following case
            else {
                // Updating the line state
                sumSensors += lf0 ? -2: 0;
                sumSensors += lf1 ? -1: 0;
                sumSensors += lf2 ? 0: 0;
                sumSensors += lf3 ? 1: 0;
                sumSensors += lf4 ? 2: 0;

                if (sumSensors > 0) {
                    if (DEBUG && LINE_DETECTION_UPDATE_AGENT_DEBUG) { Serial.println("LINE IS RIGHT"); }
                    state->lineState = RIGHT;

                } else if (sumSensors < 0) {
                    if (DEBUG && LINE_DETECTION_UPDATE_AGENT_DEBUG) { Serial.println("LINE IS LEFT"); }
                    state->lineState = LEFT;

                } else {
                    if (DEBUG && LINE_DETECTION_UPDATE_AGENT_DEBUG) { Serial.println("LINE IS CENTER"); }
                    state->lineState = CENTER;
                }
            }
        }
    }
}