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
    if(this->state->emergencyStop) {
        stopMoving();
    } else if(this->state->robotState != ARMED) {
        stopMoving();
    } else if(this->state->finalTable == COMPLETED) {
        stopMoving();
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

void NavigationActionAgent::configCircleOrientation() {
    if(this->state->circleOrientation == WEST) {
        turnLeft();
    } else if(this->state->circleOrientation == SOUTHWEST) {
        turnLeft();
    } else if(this->state->circleOrientation == SOUTH) {
        turnRight();
    } else if(this->state->circleOrientation == SOUTHEAST) {
        turnRight();
    } else if(this->state->circleOrientation == EAST) {
        turnRight();
    } else if(this->state->circleOrientation == NORTHEAST) {
        goStraightRight();
    } else if(this->state->circleOrientation == NORTH) {
        goStraight();
    } else if(this->state->circleOrientation == NORTHWEST) {
        goStraightLeft();
    } else {
        stopMoving();
    }
}

void NavigationActionAgent::configLineFollowing() {
    if(this->state->lineState == LEFT) {
        turnLeft();
    } else if(this->state->lineState == RIGHT) {
        turnRight();
    } else if(this->state->lineState == CENTER) {
        goStraight();
    } else {
        stopMoving();
    }
}

void NavigationActionAgent::configTicTacDropping() {
    stopMoving();
}

void NavigationActionAgent::configNorthWestEntity() {
    // TODO: less naive implementation
    turnRight();
}

void NavigationActionAgent::configNorthEntity() {
    // TODO: less naive implementation
    turnLeft();
}

void NavigationActionAgent::configNorthEastEntity() {
    // TODO: less naive implementation
    turnLeft();
}

void NavigationActionAgent::configureDefault() {
    goStraightLeft();
}

void NavigationActionAgent::turnLeft() {
    // Turn the robot left
    this->leftSpeed = ROBOT_SPEED; 
    this->rightSpeed = 0; 
    this->leftForward = true;
    this->rightForward = true;
}

void NavigationActionAgent::goStraightLeft() {
    // Go straight forward but deviate to the left
    this->leftSpeed = ROBOT_SPEED / 2; 
    this->rightSpeed = ROBOT_SPEED; 
    this->leftForward = true;
    this->rightForward = true;
}

void NavigationActionAgent::goStraight() {
    // Go straight forward
    this->leftSpeed = ROBOT_SPEED; 
    this->rightSpeed = ROBOT_SPEED; 
    this->leftForward = true;
    this->rightForward = true;
}

void NavigationActionAgent::goStraightRight() {
    // Go straight forward but deviate to the left
    this->leftSpeed = ROBOT_SPEED; 
    this->rightSpeed = ROBOT_SPEED / 2; 
    this->leftForward = true;
    this->rightForward = true;
}

void NavigationActionAgent::turnRight() {
    // Turn the robot right
    this->leftSpeed = 0; 
    this->rightSpeed = ROBOT_SPEED; 
    this->leftForward = true;
    this->rightForward = true;
}

void NavigationActionAgent::stopMoving() {
    // Stop the robot
    this->leftSpeed = 0; 
    this->rightSpeed = 0; 
    this->leftForward = true;
    this->rightForward = true;
}

