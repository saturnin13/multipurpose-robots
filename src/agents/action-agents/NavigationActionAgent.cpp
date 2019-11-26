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

// TODO: Add printing for each state of the robot so that we can debug and we know in which case the robot is.
void NavigationActionAgent::enact() {

    // This agent is only active if we are armed and not in an emergency
    if(this->state->emergencyStop) {
        Serial.println("ESTOP");
        stopMoving();
    } else if(this->state->robotState != ARMED) {
        Serial.println("NOT Armed");
        stopMoving();
    } else if(this->state->finalTable == COMPLETED) {
        Serial.println("COMPLETED");
        stopMoving();
    } else {
        // The order of the condition is representative of the priority of actions
        if(this->state->northWestEntity != FLAT) {
            // Avoid the north west entity
            configNorthWestEntity();
        } else if(this->state->northEntity != FLAT) {
            // Avoid the north entity
            configNorthEntity();
        } else if(this->state->northEastEntity != FLAT) {
            // Avoid the north east entity
            configNorthEastEntity();
        } else if(this->state->lineFollowingTable == COMPLETED && this->state->circleDirection != UNKNOWN) {
            // Go to circle
            configCircleDirection();
        } else if(this->state->lineFollowingTable == CURRENT) {
            // Follow line
            configLineFollowing();
        } else if(this->state->ticTacState == CURRENT) {
            // Stop if dropping
            configTicTacDropping();
        } else {
            // Set the default configurations
            configureDefault();
        }
    }

    // Set motors
    Serial.print("LEFT: ");Serial.print(this->leftSpeed);
    Serial.print(" , RIGHT: ");Serial.println(this->rightSpeed);
    this->leftMotor->configure(this->leftForward, this->leftSpeed);
    this->rightMotor->configure(this->rightForward, this->rightSpeed);
    
    // Enact motors after all
    this->leftMotor->enact();
    this->rightMotor->enact();
}

/********************
 * Configuration of the navigation for each different state of the robot
*********************/

void NavigationActionAgent::configCircleDirection() {
    if(this->state->circleDirection == WEST) {
        turnLeft();
    } else if(this->state->circleDirection == SOUTHWEST) {
        turnLeft(); //better turnLeftSpot
    } else if(this->state->circleDirection == SOUTH) {
        turnRight(); //better goReverse
    } else if(this->state->circleDirection == SOUTHEAST) {
        turnRight(); //better turnRightSpot
    } else if(this->state->circleDirection == EAST) {
        turnRight();
    } else if(this->state->circleDirection == NORTHEAST) {
        goStraightRight();
    } else if(this->state->circleDirection == NORTH) {
        goStraight();
    } else if(this->state->circleDirection == NORTHWEST) {
        goStraightLeft();
    } else {
        stopMoving();
    }
}

void NavigationActionAgent::configLineFollowing() {
    if(this->state->lineState == LEFT) {
        turnLeft(); //better turnLeftStraight
    } else if(this->state->lineState == RIGHT) {
        turnRight(); //better turnRightStraight
    } else if(this->state->lineState == CENTER) {
        goStraight();
    } else {
        stopMoving(); //todo something else, otherwise dead lock
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
    goStraight();
    //goStraightLeft();
}

/********************
 * Basic navigation direction functions
*********************/
void NavigationActionAgent::goStraight() {
    // Go straight forward
    this->leftSpeed = ROBOT_SPEED; 
    this->rightSpeed = ROBOT_SPEED; 
    this->leftForward = true;
    this->rightForward = true;
}

void NavigationActionAgent::goReverse() {
    // Go straight forward
    this->leftSpeed = ROBOT_SPEED; 
    this->rightSpeed = ROBOT_SPEED; 
    this->leftForward = false;
    this->rightForward = false;
}

void NavigationActionAgent::goStraightLeft() {
    // Go straight forward but deviate to the left
    this->leftSpeed = ROBOT_SPEED / 1.3; // TODO: put in constant and do the same for straightright
    this->rightSpeed = ROBOT_SPEED; 
    this->leftForward = true;
    this->rightForward = true;
}

void NavigationActionAgent::turnLeft() {
    // Turn the robot left
    this->leftSpeed = 0; 
    this->rightSpeed = ROBOT_SPEED; 
    this->leftForward = true;
    this->rightForward = true;
}

void NavigationActionAgent::turnLeftSpot() {
    // Turn the robot left on the spot
    this->leftSpeed = ROBOT_SPEED; 
    this->rightSpeed = ROBOT_SPEED; 
    this->leftForward = false;
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

void NavigationActionAgent::turnRightSpot() {
    // Turn the robot right on the spot
    this->leftSpeed = ROBOT_SPEED; 
    this->rightSpeed = ROBOT_SPEED; 
    this->leftForward = true;
    this->rightForward = false;
}

void NavigationActionAgent::stopMoving() {
    // Stop the robot
    this->leftSpeed = 0; 
    this->rightSpeed = 0; 
    this->leftForward = true;
    this->rightForward = true;
}

