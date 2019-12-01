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
    if(DEBUG && INCLINE_UPDATE_AGENT_DEBUG){Serial.print("\nInclineUpdateAgent: ");}
// If we are not armed, we should not do anything here.
    if (this->state->robotState != ARMED) {
        if(DEBUG && INCLINE_UPDATE_AGENT_DEBUG){Serial.println ("NOT ARMED, RETURNING");}
return;
    }

    double x = this->imu->xAngle;
    double y = this->imu->yAngle;

    if ((x < (DEGREES_CIRCLES - INCLINE_THRESHOLD) && x > INCLINE_THRESHOLD) ||
        (y < (DEGREES_CIRCLES - INCLINE_THRESHOLD) && y > INCLINE_THRESHOLD)) {
        // The threshold is exceeded.
        if(DEBUG && INCLINE_UPDATE_AGENT_DEBUG){Serial.println("INCLINE DETECTED");}
unsigned long now = millis();

        if (!this->inclineDetected) {
            this->thresholdExceededTime = now;
            this->inclineDetected = true;
        }

        if (this->state->incline == UNSEEN &&
            now - this->thresholdExceededTime > INCLINE_DETECTION_DELAY) {
            if(DEBUG && INCLINE_UPDATE_AGENT_DEBUG){Serial.println("SETTING INCLINE TO CURRENT");}
this->state->incline = CURRENT;
        }
    } else {
        if(DEBUG && INCLINE_UPDATE_AGENT_DEBUG){Serial.println("NO INCLINE DETECTED");}
// The threshold is not/no longer exceeded.
        if (this->state->incline == CURRENT) {
            if(DEBUG && INCLINE_UPDATE_AGENT_DEBUG){Serial.println("SETTING INCLINE TO COMPLETED");}
this->state->incline = COMPLETED;
            this->inclineDetected = false;
        }
    }
}