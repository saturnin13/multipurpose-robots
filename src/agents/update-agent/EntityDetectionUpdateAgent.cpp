#include <Arduino.h>

#include "EntityDetectionUpdateAgent.hpp"

EntityDetectionUpdateAgent::EntityDetectionUpdateAgent(
    State *state, UltrasonicSensor *usSW, UltrasonicSensor *usNW,
    UltrasonicSensor *usWForward, UltrasonicSensor *usNForward,
    UltrasonicSensor *usNE)
    : UpdateAgent(state), usSW(usSW), usNW(usNW), usWForward(usWForward),
      usNForward(usNForward), usNE(usNE) {}

void EntityDetectionUpdateAgent::update() {
    if (DEBUG && ENTITY_DETECTION_UPDATE_AGENT_DEBUG) {
        Serial.print("\nEntityDetectionUpdateAgent: ");
    }

    // Get sensor values
    bool usWIsObstacle = usWForward->distance < US_OBSTACLE_THRESHOLD;
    bool usNIsObstacle = usNForward->distance < US_OBSTACLE_THRESHOLD;

    bool usWIsEdge = usSW->distance > US_EDGE_THRESHOLD;
    bool usNWIsEdge = usNW->distance > US_EDGE_THRESHOLD;
    bool usNEIsEdge = usNE->distance > US_EDGE_THRESHOLD;

    // Checking the W side
    if (usWIsEdge) {
        if (DEBUG && ENTITY_DETECTION_UPDATE_AGENT_DEBUG) {
            Serial.println("W EDGE");
        }
        this->state->westEntity = EDGE;
    } else if (usWIsObstacle) {
        if (DEBUG && ENTITY_DETECTION_UPDATE_AGENT_DEBUG) {
            Serial.println("W OBSTACLE");
        }
        this->state->westEntity = OBSTACLE;
    } else {
        if (DEBUG && ENTITY_DETECTION_UPDATE_AGENT_DEBUG) {
            Serial.println("W FLAT");
        }
        this->state->westEntity = FLAT;
    }

    // Checking the NW side
    if (usNWIsEdge) {
        if (DEBUG && ENTITY_DETECTION_UPDATE_AGENT_DEBUG) {
            Serial.println("NW EDGE");
        }
        this->state->northWestEntity = EDGE;
    } else {
        if (DEBUG && ENTITY_DETECTION_UPDATE_AGENT_DEBUG) {
            Serial.println("NW FLAT");
        }
        this->state->northWestEntity = FLAT;
    }

    // Checking the N side
    if (usNEIsEdge && usNWIsEdge) {
        if (DEBUG && ENTITY_DETECTION_UPDATE_AGENT_DEBUG) {
            Serial.println("N EDGE");
        }
        this->state->northEntity = EDGE;
    } else if (usNIsObstacle) {
        if (DEBUG && ENTITY_DETECTION_UPDATE_AGENT_DEBUG) {
            Serial.println("N OBSTACLE");
        }
        this->state->northEntity = OBSTACLE;
    } else {
        if (DEBUG && ENTITY_DETECTION_UPDATE_AGENT_DEBUG) {
            Serial.println("N FLAT");
        }
        this->state->northEntity = FLAT;
    }

    // Checking the NE side
    if (usNEIsEdge) {
        if (DEBUG && ENTITY_DETECTION_UPDATE_AGENT_DEBUG) {
            Serial.println("NE EDGE");
        }
        this->state->northEastEntity = EDGE;
    } else {
        if (DEBUG && ENTITY_DETECTION_UPDATE_AGENT_DEBUG) {
            Serial.println("NE FLAT");
        }
        this->state->northEastEntity = FLAT;
    }
}