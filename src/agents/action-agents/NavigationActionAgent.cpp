#include "NavigationActionAgent.hpp"
#include "ActionAgent.hpp"
#include <Arduino.h>


NavigationActionAgent::NavigationActionAgent(State* state, Motor* leftMotor, Motor* rightMotor)
: ActionAgent(state), leftMotor(leftMotor), rightMotor(rightMotor) {

    this->leftSpeed = 0; 
    this->rightSpeed = 0; 
    this->leftForward = true;
    this->rightForward = true;
}

void NavigationActionAgent::enact() {

    // This agent is only active if we are armed and not in an emergency
    if(this->state->emergencyStop || this->state->robotState != ARMED) {
        this->leftSpeed = 0; 
        this->rightSpeed = 0; 
    } else {
        if(this->state->circleOrientation != UNKNOWN) {
            // Go to circle
            configCircleOrientation();
        } else if(this->state->lineFollowingTable == CURRENT && this->state->lineState != LOST) {
            // Follow line
            // Case of being lost on line following table has not been implemented, it could be done
            configLineFollowing();
        } else if(this->state->ticTacState == DROPPING) {
            // Stop if dropping
            configTicTacDropping();
        } else if(this->state->northWestEntity != FLAT) {
            // Avoid the north west entity
            configNorthWestEntity();
        } else if(this->state->northEntity != FLAT) {
            // Avoid the north entity
            configNorthEntity();
        } else if(this->state->northEastEntity != FLAT) {
            // Avoid the north east entity
            configNorthEastEntity();
        } else {
            // Set the default configurations
            configureDefault();
        }
    }

    // Set motors
    this->leftMotor->configure(this->leftForward, this->leftSpeed);
    this->rightMotor->configure(this->rightForward, this->rightSpeed);
    
    // Enact motors after all
    this->leftMotor->enact();
    this->rightMotor->enact();
}

void configCircleOrientation() {
    // TODO
}

void configLineFollowing() {
    // TODO
}

void configTicTacDropping() {
    // TODO
}

void configNorthWestEntity() {
    // TODO
}

void configNorthEntity() {
    // TODO
}

void configNorthEastEntity() {
    // TODO
}

void configureDefault() {
    // TODO
}
