#include "NavigationActionAgent.hpp"
#include "ActionAgent.hpp"
#include <Arduino.h>


NavigationActionAgent::NavigationActionAgent(State* state, Motor* leftMotor, Motor* rightMotor)
: ActionAgent(state), leftMotor(leftMotor), rightMotor(rightMotor) {

    this->leftSpeed = 0; 
    this->rightSpeed = 0; 
    this->leftForward = true;
    this->rightForward = true;

    this->priority = PRIO3;
}

void NavigationActionAgent::enact() {

    //this agent is only active if we are armed
    if(this->state->robotState != ARMED)
    {
        return;
    }

    //read state

    //most important
    if(this->state->emergencyStop) {
        this->priority = PRIO1;
        this->leftSpeed = 0; 
        this->rightSpeed = 0; 
    }

    //go to circle if there
    this->state->circleOrientation;

    //follow line if there
    this->state->lineFollowingTable;
    this->state->lineState;

    //stop if dropping
    this->state->ticTacState;

    //make sure we do not drive in direction of edges or obstacles
    this->state->northEastEntity;
    this->state->northWestEntity;
    this->state->northEntity;
    this->state->southWestEntity;

    //vales accordingly to things before
    this->leftSpeed = 0; 
    this->rightSpeed = 0; 
    this->leftForward = true;
    this->rightForward = true;

    //set motors
    this->leftMotor->configure(this->leftForward, this->leftSpeed);
    this->rightMotor->configure(this->rightForward, this->rightSpeed);

    //enact motors after all
    this->leftMotor->enact();
    this->rightMotor->enact();
}