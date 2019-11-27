#include <Arduino.h>

#include "EntityDetectionUpdateAgent.hpp"


EntityDetectionUpdateAgent::EntityDetectionUpdateAgent(State* state, UltrasonicSensor* usSW, UltrasonicSensor* usNW, UltrasonicSensor* usNNWForward, UltrasonicSensor* usWForward,
    UltrasonicSensor* usNNEForward, UltrasonicSensor* usNE): UpdateAgent(state), usSW(usSW), usNW(usNW), usNNWForward(usNNWForward), usWForward(usWForward), usNNEForward(usNNEForward), usNE(usNE) {

}

void EntityDetectionUpdateAgent::update() {

    // Get sensor values
    //DISCUSS: what if a value is suddenly 0?
    bool usNNWIsObstacle = usNNWForward->distance < US_OBSTACLE_THRESHOLD;
    bool usWIsObstacle = usWForward->distance < US_OBSTACLE_THRESHOLD;
    bool usNNEIsObstacle = usNNEForward->distance < US_OBSTACLE_THRESHOLD;

    //TODO improve outlier detection (here every value over US_EDGE_MAX will not be considered)
    bool usWIsEdge = usSW->distance > US_EDGE_THRESHOLD && usSW->distance < US_EDGE_MAX;
    bool usNWIsEdge = usNW->distance > US_EDGE_THRESHOLD && usNW->distance < US_EDGE_MAX;
    bool usNEIsEdge = usNE->distance > US_EDGE_THRESHOLD && usNE->distance < US_EDGE_MAX;
    
    //TODO remove
    if(usWIsEdge || usNWIsEdge || usNEIsEdge) {
        Serial.println("EDGE DETECTED");
        Serial.print("EDGE (SW,NW,NE): ");Serial.print(usWIsEdge);Serial.print(usNWIsEdge);Serial.println(usNEIsEdge);
        this->state->emergencyStop = true;
    }

    // Checking the W side
    if(usWIsObstacle) {
        this->state->westEntity = OBSTACLE;
    } else if(usWIsEdge) {
        this->state->westEntity = EDGE;
    } else {
        this->state->westEntity = FLAT;
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