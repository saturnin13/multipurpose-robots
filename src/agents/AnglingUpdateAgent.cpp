#include "AnglingUpdateAgent.hpp"

#define ANGLE_THRESHOLD 10
#define DEGREES_CIRCLES 360

AnglingUpdateAgent::AnglingUpdateAgent(State &state, IMUSensor imu) {
    this->state = state;
    this->imu = imu;
}

void AnglingUpdateAgent::update() {
    double x = imu.xAngle
    double y = imu.yAngle
    
    if((x < DEGREES_CIRCLES - ANGLE_THRESHOLD || x > ANGLE_THRESHOLD) && 
        (y < DEGREES_CIRCLES - ANGLE_THRESHOLD || y > ANGLE_THRESHOLD)) {
        state.incline = CURRENT
    }

    if((x > DEGREES_CIRCLES - ANGLE_THRESHOLD || x < ANGLE_THRESHOLD) && 
        (y > DEGREES_CIRCLES - ANGLE_THRESHOLD || y < ANGLE_THRESHOLD) && 
        state.incline == CURRENT) {
        state.incline = COMPLETED
    }
}