#include "AnglingUpdateAgent.hpp"

#define ANGLE_THRESHOLD 10

AnglingUpdateAgent::AnglingUpdateAgent(State* state, IMUSensor* imu): UpdateAgent(state), imu(imu) {

}

void AnglingUpdateAgent::update() {
    double x = this->imu->xAngle;
    double y = this->imu->yAngle;
    
    if((x < (DEGREES_CIRCLES - ANGLE_THRESHOLD) && x > ANGLE_THRESHOLD) ||
        (y < (DEGREES_CIRCLES - ANGLE_THRESHOLD) && y > ANGLE_THRESHOLD)) {
        this->state->incline = CURRENT;
    } else if (this->state->incline == CURRENT) {
        this->state->incline = COMPLETED;
        //TODO remove and put to TICTACUPDATEAGENT
        this->state->ticTacState = DROPPING;
    }
}