#include <Arduino.h>

#include "LoopDetectionUpdateAgent.hpp"

#define DEGREE_QUADRANT 90

LoopDetectionUpdateAgent::LoopDetectionUpdateAgent(State *state, IMUSensor *imu)
    : UpdateAgent(state), imu(imu) {
    quadrant = 0;
}

void LoopDetectionUpdateAgent::update() {
    if (DEBUG && LOOP_DETECTION_UPDATE_AGENT_DEBUG) {
        Serial.print("\nLoopDetectionUpdateAgent: ");
    }
    // Get sensor values
    double newAngle = imu->zAngle;

    // We are checking if the angle of the robot has moved by more than 90
    // degrees and then we are incrementing or decrementing accordingly
    double angleDifference =
        min(abs(newAngle - previousQuadrant),
            DEGREES_CIRCLE - abs(newAngle - previousQuadrant));

    if (angleDifference > DEGREE_QUADRANT) {
        if (DEBUG && LOOP_DETECTION_UPDATE_AGENT_DEBUG) {
            Serial.println("REACHING NEW QUADRANT");
        }
        // Case where the quadrant is bigger than the previous one
        if (newAngle - previousQuadrant == angleDifference) {
            if (DEBUG && LOOP_DETECTION_UPDATE_AGENT_DEBUG) {
                Serial.println("QUADRANT ON THE RIGHT");
            }
            this->previousQuadrant = newAngle;
            quadrant++;

            // Case where the quadrant is smaller than the previous one
        } else if (DEGREES_CIRCLE - abs(newAngle - previousQuadrant) ==
                   angleDifference) {
            if (DEBUG && LOOP_DETECTION_UPDATE_AGENT_DEBUG) {
                Serial.println("QUADRANT ON THE LEFT");
            }
            this->previousQuadrant = newAngle;
            quadrant--;
        }
    }

    state->totalTurnedQuadrants = quadrant;
}
