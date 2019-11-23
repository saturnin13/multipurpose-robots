#include "EStopUpdateAgent.hpp"

EStopUpdateAgent::EStopUpdateAgent(State* state, ros::NodeHandle* nh, ros::Subscriber<std_msgs::Empty> subES)
: UpdateAgent(state), nh(nh), subES(subES) {
    this->eStopRequested = false;
    this->nh->subscribe(this->subES);
}

void EStopUpdateAgent::update() {
    update(false);
}


void EStopUpdateAgent::update(bool eStopRequested) {
    //add estop button later
    //if(this->b->pressed && this->state->robotState == ARMED) {
    //    this->state->emergencyStop = true;
    //}

    if(!this->eStopRequested && eStopRequested) {
        this->eStopRequested = eStopRequested;
        this->state->emergencyStop = true;
    }

    //make sure we can restart it
    //therefore new message, then reset
}