#include <Arduino.h>

#include "EntityDetectionUpdateAgent.hpp"


EntityDetectionUpdateAgent::EntityDetectionUpdateAgent(State* state, LineFollowerSensor* lf, UltrasonicSensor* usSW, UltrasonicSensor* usNW, UltrasonicSensor* usNWForward, UltrasonicSensor* usNForward, 
    UltrasonicSensor* usNEForward, UltrasonicSensor* usNE): UpdateAgent(state), lf(lf), usSW(usSW), usNW(usNW), usNWForward(usNWForward), usNForward(usNForward), usNEForward(usNEForward), usNE(usNE) {

}

void EntityDetectionUpdateAgent::update() {
    // Get sensor values
    //TODO hack
    bool usNWIsObstacle = false;//usNWForward->distance < US_OBSTACLE_THRESHOLD;
    bool usNIsObstacle = false;//usNForward->distance < US_OBSTACLE_THRESHOLD;
    bool usNEIsObstacle = false;//usNEForward->distance < US_OBSTACLE_THRESHOLD;

    bool usSWIsEdge = usSW->distance > US_EDGE_THRESHOLD;
    bool usNWIsEdge = usNW->distance > US_EDGE_THRESHOLD;
    bool usNEIsEdge = usNE->distance > US_EDGE_THRESHOLD;

    bool lfFrontIsEdge = lf->lineDetected2;

    if(lfFrontIsEdge || usSWIsEdge || usNWIsEdge || usNEIsEdge) {
        Serial.print("EDGES ");Serial.print(usSWIsEdge);Serial.print(usNWIsEdge);Serial.println(usNEIsEdge);
        this->state->emergencyStop = true;
    }

    // Checking the SW side
    if(usSWIsEdge) {
        this->state->southWestEntity = EDGE;
    } else {
        this->state->southWestEntity = FLAT;
    }

    // Checking the NW side
    if(usNWIsEdge) {
        this->state->northWestEntity = EDGE;
    } else if(usNWIsObstacle) {
        this->state->northWestEntity = OBSTACLE;
    } else {
        this->state->northWestEntity = FLAT;
    }

    // Checking the N side
    if(usNIsObstacle) {
        this->state->northEntity = OBSTACLE;
    } else {
        this->state->northEntity = FLAT;
    }

    // Checking the NE side
    if(usNEIsEdge) {
        this->state->northEastEntity = EDGE;
    } else if(usNEIsObstacle) {
        this->state->northEastEntity = OBSTACLE;
    } else {
        this->state->northEastEntity = FLAT;
    }
}