#include <Arduino.h>

#include "EntityDetectionUpdateAgent.hpp"


EntityDetectionUpdateAgent::EntityDetectionUpdateAgent(State* state, LineFollowerSensor* lf, UltrasonicSensor* usSW, UltrasonicSensor* usNW, UltrasonicSensor* usNNWForward, UltrasonicSensor* usNForward,
    UltrasonicSensor* usNNEForward, UltrasonicSensor* usNE): UpdateAgent(state), lf(lf), usSW(usSW), usNW(usNW), usNNWForward(usNNWForward), usNForward(usNForward), usNNEForward(usNNEForward), usNE(usNE) {

}

void EntityDetectionUpdateAgent::update() {
    // Get sensor values
    //TODO hack
    bool usNNWIsObstacle = false;//usNNWForward->distance < US_OBSTACLE_THRESHOLD;
    bool usNIsObstacle = false;//usNForward->distance < US_OBSTACLE_THRESHOLD;
    bool usNNEIsObstacle = false;//usNNEForward->distance < US_OBSTACLE_THRESHOLD;

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
    } else if(usNNWIsObstacle) {
        this->state->northWestEntity = OBSTACLE;
    } else {
        this->state->northWestEntity = FLAT;
    }

    // Checking the N side
    if(usNIsObstacle) {
        this->state->northEntity = OBSTACLE;
    } else if(usNEIsEdge && usNWIsEdge) {
        this->state->northEntity = EDGE;
    } else {
        this->state->northEntity = FLAT;
    }

    // Checking the NE side
    if(usNEIsEdge) {
        this->state->northEastEntity = EDGE;
    } else if(usNNEIsObstacle) {
        this->state->northEastEntity = OBSTACLE;
    } else {
        this->state->northEastEntity = FLAT;
    }
}