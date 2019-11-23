#include <Arduino.h>

#include "EntityDetectionUpdateAgent.hpp"
#include "../../Constants.hpp"


EntityDetectionUpdateAgent::EntityDetectionUpdateAgent(State* state, UltrasonicSensor* usSW, UltrasonicSensor* usNW, UltrasonicSensor* usNNW, UltrasonicSensor* usN, 
    UltrasonicSensor* usNNE, UltrasonicSensor* usNE): UpdateAgent(state), usSW(usSW), usNW(usNW), usNNW(usNNW), usN(usN), usNNE(usNNE), usNE(usNE) {

}

void EntityDetectionUpdateAgent::update() {
    // Get sensor values
    bool usNNWIsObstacle = usNNW->distance < USOBSTACLETHRESHOLD;
    bool usNIsObstacle = usN->distance < USOBSTACLETHRESHOLD;
    bool usNNEIsObstacle = usNNE->distance < USOBSTACLETHRESHOLD;

    bool usSWIsEdge = usSW->distance > USEDGETHRESHOLD;
    bool usNWIsEdge = usNW->distance > USEDGETHRESHOLD;
    bool usNEIsEdge = usNE->distance > USEDGETHRESHOLD;

    // Checking the SW side
    if(usSWIsEdge) {
        this->state->southWestEntity = EDGE;
    } else {
        this->state->southWestEntity = FLAT;
    }

    // Checking the NW side
    if(usNWIsEdge) {
        this->state->northWestEntity = EDGE;
    } else {
        this->state->northWestEntity = FLAT;
    }

    // Checking the N side
    if(usNNWIsObstacle || usNIsObstacle || usNNEIsObstacle) {
        this->state->northEntity = OBSTACLE;
    } else {
        this->state->northEntity = FLAT;
    }

    // Checking the NE side
    if(usNEIsEdge) {
        this->state->northEastEntity = EDGE;
    } else {
        this->state->northEastEntity = FLAT;
    }
}