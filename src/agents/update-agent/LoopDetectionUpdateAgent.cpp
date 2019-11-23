#include <Arduino.h>

#include "LoopDetectionUpdateAgent.hpp"

#define DEGREE_QUADRANT 90

LoopDetectionUpdateAgent::LoopDetectionUpdateAgent(State* state, IMUSensor* imu): UpdateAgent(state), imu(imu) {
    quadrant = 0;
}

void LoopDetectionUpdateAgent::update() {
    // Get sensor values
    double newAngle = imu->zAngle;

    // We are checking if the angle of the robot has moved by more than 90 degrees and then we are incrementing or decrementing accordingly
    double angleDifference = min(abs(newAngle - previousQuadrant), DEGREES_CIRCLES - abs(newAngle - previousQuadrant));

    if(angleDifference > DEGREE_QUADRANT) {
        
        // Case where the quadrant is bigger than the previous one
        if(newAngle - previousQuadrant == angleDifference) {
            quadrant++;

        // Case where the quadrant is smaller than the previous one
        } else if(DEGREES_CIRCLES - abs(newAngle - previousQuadrant) == angleDifference) {
            quadrant--;
        }
    }

    state->totalTurnedQuadrants = DEGREE_QUADRANT * quadrant;
}
