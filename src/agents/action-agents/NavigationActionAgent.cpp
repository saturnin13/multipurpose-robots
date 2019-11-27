#include "NavigationActionAgent.hpp"
#include "ActionAgent.hpp"
#include <Arduino.h>

#define STATE_PRINTING 0


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
        if(STATE_PRINTING){Serial.println("NO MOVING: ESTOP");}
        stopMoving();

    } else if(this->state->robotState != ARMED) {
        if(STATE_PRINTING){Serial.println("NO MOVING: NOT Armed");}
        stopMoving();

    } else if(!this->state->move) {
        if(STATE_PRINTING){Serial.println("NO MOVING: TESTING");}
        stopMoving();

    } else if(this->state->finalTable == COMPLETED) {
        if(STATE_PRINTING){Serial.println("NO MOVING: COMPLETED");}
        stopMoving();

    } else {

        // The order of the condition is representative of the priority of actions
        if(this->state->northWestEntity != FLAT) {
            if(STATE_PRINTING){Serial.println("AVOIDING NW");}
            // Avoid the north west entity
            configNorthWestEntity();

        } else if(this->state->northEntity != FLAT) {
            if(STATE_PRINTING){Serial.println("AVOIDING N");}
            // Avoid the north entity
            configNorthEntity();

        } else if(this->state->northEastEntity != FLAT) {
            if(STATE_PRINTING){Serial.println("AVOIDING NE");}
            // Avoid the north east entity
            configNorthEastEntity();

        } else if(this->state->westEntity != FLAT) {
            // Avoid the west entity
            configWestEntity();

        } else if(this->state->lineFollowingTable == COMPLETED && this->state->circleDirection != UNKNOWN) {
            if(STATE_PRINTING){Serial.println("GO TO CIRCLE");}
            // Go to circle
            configCircleDirection();

        } else if(this->state->lineFollowingTable == CURRENT) {
            if(STATE_PRINTING){Serial.println("FOLLOW LINE");}
            // Follow line
            configLineFollowing();

        } else if(this->state->ticTacState == CURRENT) {
            if(STATE_PRINTING){Serial.println("DROP TICTAC Driving");}
            // Stop if dropping
            configTicTacDropping();

        } else {
            if(STATE_PRINTING){Serial.println("DEFAULT Driving");}
            // Set the default configurations
            configureDefault();
        }
    }

    // Set motors
    unsigned int now = millis();
    if(now % 1000 > 990) {
    Serial.print("LEFT: ");Serial.print(this->leftSpeed);
    Serial.print(" , RIGHT: ");Serial.println(this->rightSpeed);
    }
    this->leftMotor->configure(this->leftForward, 12);//this->leftSpeed);
    this->rightMotor->configure(this->rightForward, 12);//this->rightSpeed);
    
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
        Serial.println("GOING LEFT");
        turnLeft(); //better turnLeftStraight
    } else if(this->state->lineState == RIGHT) {
        Serial.println("GOING RIGHT");
        turnRight(); //better turnRightStraight
    } else if(this->state->lineState == CENTER) {
        Serial.println("GOING CENTER");
        goStraight();
    } else {
        stopMoving(); //TODO something else, otherwise dead lock
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
    // TODO: will this work? less naive implementation
    turnLeft();
}

void NavigationActionAgent::configNorthEastEntity() {
    // TODO: if there is something NE, we wont fit between less naive implementation
    turnLeft();
}

void NavigationActionAgent::configWestEntity() {
    // TODO: less naive implementation
    goStraightRight();
}

void NavigationActionAgent::configureDefault() {
    goStraight();
    //TODO change back after testing
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
    this->leftSpeed = ROBOT_SPEED / RATIO_FAST_TO_SLOW_MOTOR; // TODO: put in constant and do the same for straightright
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
    this->leftSpeed = ROBOT_SPEED / RATIO_FAST_TO_SLOW_MOTOR; 
    this->rightSpeed = ROBOT_SPEED; 
    this->leftForward = true;
    this->rightForward = true;
}

void NavigationActionAgent::turnRight() {
    // Turn the robot right
    this->leftSpeed = ROBOT_SPEED; 
    this->rightSpeed = 0; 
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

