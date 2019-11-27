#include "AnglingUpdateAgent.hpp"
#include <Arduino.h>

#define ANGLE_THRESHOLD 12.0

AnglingUpdateAgent::AnglingUpdateAgent(State *state, IMUSensor *imu)
    : UpdateAgent(state), imu(imu) {}

void AnglingUpdateAgent::update() {

    // if we are not armed, we should not do anything here.
    if (this->state->robotState != ARMED) {
        return;
    }

    double x = this->imu->xAngle;
    double y = this->imu->yAngle;

    if ((x < (DEGREES_CIRCLES - ANGLE_THRESHOLD) && x > ANGLE_THRESHOLD) ||
        (y < (DEGREES_CIRCLES - ANGLE_THRESHOLD) && y > ANGLE_THRESHOLD)) {
        this->state->incline = CURRENT;

    } else if (this->state->incline == CURRENT) {
        this->state->incline = COMPLETED;
    }
}