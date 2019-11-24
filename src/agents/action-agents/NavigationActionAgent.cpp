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
        stopMoving();
    } else if(this->state->robotState != ARMED) {
        stopMoving();
    } else if(this->state->finalTable == COMPLETED) {
        stopMoving();
    //} if(this->state->ticTacState == DROPPING) {
        // Stop if dropping
        //configTicTacDropping();
    } else {
        //case we do not have to stop for some reason, so we think about which way we go
        goStraight();
        /*
        bool edgeLeft = this->state->southWestEntity == EDGE || this->state->northWestEntity == EDGE;
        bool edgeFront = this->state->northWestEntity == EDGE || this->state->northEastEntity == EDGE;
        if(edgeLeft && !edgeFront) {
            goStraightRight();
        } else if (edgeLeft && edgeFront) {
            turnRightSpot();
        } else if (!edgeLeft && !edgeFront) {
            bool obstacleLeft = this->state->southWestEntity == OBSTACLE || this->state->northWestEntity == OBSTACLE;
            bool obstacleFront = this->state->northWestEntity == OBSTACLE || this->state->northEastEntity == OBSTACLE;

            //case we dont have edges, so we want to avoid obstacles
            if(obstacleLeft && !obstacleFront) {
                goStraightRight();
            } else if (obstacleLeft && obstacleFront) {
                turnRightSpot();
            } else if (!obstacleLeft && !obstacleFront) {

                //case we dont have edges or obstacles here, so we care about the rest
                if(this->state->lineFollowingTable == CURRENT) {
                    // Follow line
                    // Case of being lost on line following table has not been implemented, it could be done
                    configLineFollowing();
                } else if (this->state->lineFollowingTable == COMPELTED && this->state->circleOrientation != UNKNOWN) {
                    // Go to circle
                    configCircleOrientation();
                } else {
                    //go straigh left/ default
                    configureDefault();
                }
            }
        }
        */

        /*if(this->state->circleOrientation != UNKNOWN) {
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
        }*/
    }

    // Set motors
    //Serial.print("LEFT: ");Serial.print(this->leftSpeed);
    //Serial.print("RIGHT: ");Serial.println(this->rightSpeed);
    this->leftMotor->configure(this->leftForward, this->leftSpeed);
    this->rightMotor->configure(this->rightForward, this->rightSpeed);
    
    // Enact motors after all
    this->leftMotor->enact();
    this->rightMotor->enact();
}

/********************
 * Configuration of the navigation for each different state of the robot
*********************/

void NavigationActionAgent::configCircleOrientation() {
    if(this->state->circleOrientation == WEST) {
        turnLeft();
    } else if(this->state->circleOrientation == SOUTHWEST) {
        turnLeft(); //better turnLeftSpot
    } else if(this->state->circleOrientation == SOUTH) {
        turnRight(); //better goReverse
    } else if(this->state->circleOrientation == SOUTHEAST) {
        turnRight(); //better turnRightSpot
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
    goStraightLeft();
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
    this->leftSpeed = ROBOT_SPEED / 2; 
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
