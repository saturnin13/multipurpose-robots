#include <Arduino.h>

#include "EntityDetectionUpdateAgent.hpp"


EntityDetectionUpdateAgent::EntityDetectionUpdateAgent(State* state, UltrasonicSensor* usSW, UltrasonicSensor* usNW, UltrasonicSensor* usNNWForward, UltrasonicSensor* usSWForward,
    UltrasonicSensor* usNNEForward, UltrasonicSensor* usNE): UpdateAgent(state), usSW(usSW), usNW(usNW), usNNWForward(usNNWForward), usSWForward(usSWForward), usNNEForward(usNNEForward), usNE(usNE) {

}

void EntityDetectionUpdateAgent::update() {
    // Get sensor values
    //TODO hack
    bool usNNWIsObstacle = false;//usNNWForward->distance < US_OBSTACLE_THRESHOLD;
    bool usSWIsObstacle = false;//usSWForward->distance < US_OBSTACLE_THRESHOLD;
    bool usNNEIsObstacle = false;//usNNEForward->distance < US_OBSTACLE_THRESHOLD;

    //TODO improve outlier detection (here every value over US_EDGE_MAX will not be considered)
    bool usSWIsEdge = usSW->distance > US_EDGE_THRESHOLD && usSW->distance < US_EDGE_MAX;
    bool usNWIsEdge = usNW->distance > US_EDGE_THRESHOLD && usNW->distance < US_EDGE_MAX;
    bool usNEIsEdge = usNE->distance > US_EDGE_THRESHOLD && usNE->distance < US_EDGE_MAX;
    
    //TODO remove
    if(usSWIsEdge || usNWIsEdge || usNEIsEdge) {
        Serial.println("EDGE DETECTED");
        Serial.print("EDGE (SW,NW,NE): ");Serial.print(usSWIsEdge);Serial.print(usNWIsEdge);Serial.println(usNEIsEdge);
        this->state->emergencyStop = true;
    }

    // Checking the SW side
    if(usSWIsObstacle) {
        this->state->southWestEntity = OBSTACLE;
    } else if(usSWIsEdge) {
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
    if(usNEIsEdge && usNWIsEdge) {
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