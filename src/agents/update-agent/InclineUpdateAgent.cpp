#include "InclineUpdateAgent.hpp"
#include <Arduino.h>

// The incline in degrees.
#define INCLINE_THRESHOLD 15.0

// How long the threshold should be exceeded before the incline is detected (in
// milliseconds).
#define INCLINE_DETECTION_DELAY 1000

InclineUpdateAgent::InclineUpdateAgent(State *state, IMUSensor *imu)
    : UpdateAgent(state), imu(imu) {}

void InclineUpdateAgent::update() {
    if(DEBUG && INCLINE_UPDATE_AGENT_DEBUG){Serial.print("\nInclineUpdateAgent: ");}

    double x = this->imu->xAngle;
    double y = this->imu->yAngle;

    // If we are not armed, we should not do anything here.
    if (this->state->robotState != ARMED) {
        if(DEBUG && INCLINE_UPDATE_AGENT_DEBUG){Serial.println ("NOT ARMED, RETURNING");}
        return;
    }

    bool hasExceededAngle = (x < (DEGREES_CIRCLE - INCLINE_THRESHOLD) && x > INCLINE_THRESHOLD) ||
                             (y < (DEGREES_CIRCLE - INCLINE_THRESHOLD) && y > INCLINE_THRESHOLD);

    if (hasExceededAngle) {
        // The threshold is exceeded.
        if (DEBUG && INCLINE_UPDATE_AGENT_DEBUG) { Serial.println("INCLINE DETECTED"); }

        if (!this->lastDetectedIsIncline) {
            this->firstInclineDetectedTime = millis();
            this->lastDetectedIsIncline = true;
        }

        if (this->state->incline == UNSEEN && millis() - this->firstInclineDetectedTime > INCLINE_DETECTION_DELAY) {
            if (DEBUG && INCLINE_UPDATE_AGENT_DEBUG) { Serial.println("SETTING INCLINE TO CURRENT");} 
            this->state->incline = CURRENT;
        }

        if(this->state->incline == COMPLETED && !this->declineDetected) {
            this->declineDetected = true;
            this->state->decline = CURRENT;
        }
    } else {
        if(DEBUG && INCLINE_UPDATE_AGENT_DEBUG){Serial.println("NO INCLINE DETECTED");}

        if (this->lastDetectedIsIncline) {
            this->firstFlatDetectedTime = millis();
            this->lastDetectedIsIncline = false;
        }

        // The threshold is not/no longer exceeded.
        if (this->state->incline == CURRENT && millis() - this->firstFlatDetectedTime > INCLINE_DETECTION_DELAY) {
            if(DEBUG && INCLINE_UPDATE_AGENT_DEBUG){Serial.println("SETTING INCLINE TO COMPLETED");}
            this->state->incline = COMPLETED;
            this->lastDetectedIsIncline = false;
        }

        if (this->state->decline == CURRENT) {
            if(DEBUG && INCLINE_UPDATE_AGENT_DEBUG){Serial.println("SETTING DECLINE TO COMPLETED");}
            this->state->decline = COMPLETED;
            this->declineDetected = false;
        }
    }
}