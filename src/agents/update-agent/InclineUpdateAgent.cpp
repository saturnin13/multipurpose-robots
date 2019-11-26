#include "InclineUpdateAgent.hpp"
#include <Arduino.h>

#define INCLINE_THRESHOLD 12.0

InclineUpdateAgent::InclineUpdateAgent(State *state, IMUSensor *imu)
    : UpdateAgent(state), imu(imu) {}

void InclineUpdateAgent::update() {

    // if we are not armed, we should not do anything here.
    if (this->state->robotState != ARMED) {
        return;
    }

    double x = this->imu->xAngle;
    double y = this->imu->yAngle;

    if ((x < (DEGREES_CIRCLES - INCLINE_THRESHOLD) && x > INCLINE_THRESHOLD) ||
        (y < (DEGREES_CIRCLES - INCLINE_THRESHOLD) && y > INCLINE_THRESHOLD)) {
        this->state->incline = CURRENT;
    } else if (this->state->incline == CURRENT) {
        this->state->incline = COMPLETED;
    }
}