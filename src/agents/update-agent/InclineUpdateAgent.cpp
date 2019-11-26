#include "InclineUpdateAgent.hpp"
#include <Arduino.h>

// The incline in degrees.
#define INCLINE_THRESHOLD 12.0

// How long the threshold should be exceeded before the incline is detected (in
// milliseconds).
#define INCLINE_DETECTION_DELAY 1000

InclineUpdateAgent::InclineUpdateAgent(State *state, IMUSensor *imu)
    : UpdateAgent(state), imu(imu) {}

void InclineUpdateAgent::update() {

    // If we are not armed, we should not do anything here.
    if (this->state->robotState != ARMED) {
        return;
    }

    double x = this->imu->xAngle;
    double y = this->imu->yAngle;

    if ((x < (DEGREES_CIRCLES - INCLINE_THRESHOLD) && x > INCLINE_THRESHOLD) ||
        (y < (DEGREES_CIRCLES - INCLINE_THRESHOLD) && y > INCLINE_THRESHOLD)) {
        // The threshold is exceeded.

        unsigned long now = millis();

        if (this->inclineDetected == false) {
            this->thresholdExceededTime = now;
            this->inclineDetected = true;
        }

        if (this->state->incline == UNSEEN &&
            now - this->thresholdExceededTime > INCLINE_DETECTION_DELAY) {
            this->state->incline = CURRENT;
        }
    } else {
        // The threshold is not/no longer exceeded.
        if (this->state->incline == CURRENT) {
            this->state->incline = COMPLETED;
            this->inclineDetected = false;
        }
    }
}