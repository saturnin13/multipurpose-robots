#include <Arduino.h>

#include "EntityDetectionUpdateAgent.hpp"


EntityDetectionUpdateAgent::EntityDetectionUpdateAgent(State* state, UltrasonicSensor* usSW, UltrasonicSensor* usNW, UltrasonicSensor* usNNWForward, UltrasonicSensor* usWForward,
    UltrasonicSensor* usNNEForward, UltrasonicSensor* usNE): UpdateAgent(state), usSW(usSW), usNW(usNW), usNNWForward(usNNWForward), usWForward(usWForward), usNNEForward(usNNEForward), usNE(usNE) {

}

void EntityDetectionUpdateAgent::update() {
    if(DEBUG && ENTITY_DETECTION_UPDATE_AGENT_DEBUG){Serial.print("\nEntityDetectionUpdateAgent: ");}
// Get sensor values
    bool usNNWIsObstacle = usNNWForward->distance < US_OBSTACLE_THRESHOLD;
    bool usWIsObstacle = usWForward->distance < US_OBSTACLE_THRESHOLD;
    bool usNNEIsObstacle = usNNEForward->distance < US_OBSTACLE_THRESHOLD;

    bool usWIsEdge = usSW->distance > US_EDGE_THRESHOLD;
    bool usNWIsEdge = usNW->distance > US_EDGE_THRESHOLD;
    bool usNEIsEdge = usNE->distance > US_EDGE_THRESHOLD;

    // Checking the W side
    if(usWIsEdge) {
        if(DEBUG && ENTITY_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("W EDGE");}
        this->state->westEntity = EDGE;
    } else if(usWIsObstacle) {
        if(DEBUG && ENTITY_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("W OBSTACLE");}
        this->state->westEntity = OBSTACLE;
    } else {
        if(DEBUG && ENTITY_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("W FLAT");}
        this->state->westEntity = FLAT;
    }

    // Checking the NW side
    if(usNWIsEdge) {
        if(DEBUG && ENTITY_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("NW EDGE");}
        this->state->northWestEntity = EDGE;
    } else if(usNNWIsObstacle) {
        if(DEBUG && ENTITY_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("NW OBSTACLE");}
        this->state->northWestEntity = OBSTACLE;
    } else {
        if(DEBUG && ENTITY_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("NW FLAT");}
        this->state->northWestEntity = FLAT;
    }

    // Checking the N side
    if(usNEIsEdge && usNWIsEdge) {
        if(DEBUG && ENTITY_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("N EDGE");}
        this->state->northEntity = EDGE;
    } else {
        if(DEBUG && ENTITY_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("N FLAT");}
        this->state->northEntity = FLAT;
    }

    // Checking the NE side
    if(usNEIsEdge) {
        if(DEBUG && ENTITY_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("NE EDGE");}
        this->state->northEastEntity = EDGE;
    } else if(usNNEIsObstacle) {
        if(DEBUG && ENTITY_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("NE OBSTACLE");}
        this->state->northEastEntity = OBSTACLE;
    } else {
        if(DEBUG && ENTITY_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("NE FLAT");}
        this->state->northEastEntity = FLAT;
    }
}