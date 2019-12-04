#include "NavigationActionAgent.hpp"
#include "ActionAgent.hpp"
#include <Arduino.h>

NavigationActionAgent::NavigationActionAgent(State* state, Motor* leftMotor, Motor* rightMotor)
: ActionAgent(state), leftMotor(leftMotor), rightMotor(rightMotor) {
    this->leftSpeed = 0; 
    this->rightSpeed = 0; 
    this->leftForward = true;
    this->rightForward = true;

    this->nextManoeuver = NO_MANOEUVER;
    this->manoeuverStartTime = 0;
}

void NavigationActionAgent::enact() {
    if(DEBUG && NAVIGATION_ACTION_AGENT_DEBUG){Serial.print("\nNavigationActionAgent: ");}

    if(this->state->emergencyStop) {
        if(DEBUG && NAVIGATION_ACTION_AGENT_DEBUG){Serial.println("E-STOP");}
        stopMoving();

    } else if(this->state->robotState != ARMED) {
        if(DEBUG && NAVIGATION_ACTION_AGENT_DEBUG){Serial.println("NOT ARMED");}
        stopMoving();

    } else if(!this->state->move) {
        if(DEBUG && NAVIGATION_ACTION_AGENT_DEBUG){Serial.println("NOT MOVING");}
        stopMoving();

    } else if(this->state->finalTable == COMPLETED) {
        if(DEBUG && NAVIGATION_ACTION_AGENT_DEBUG){Serial.println("FINAL TABLE COMPLETED");}
        stopMoving();

    } else if(this->state->ticTacState == CURRENT) {
        if(DEBUG && NAVIGATION_ACTION_AGENT_DEBUG){Serial.println("DROPPING TIC TACS");}
        stopMoving();

    } else {

        // The order of the condition is representative of the priority of actions
        if (this->state->northEntity != FLAT) {
            if(DEBUG && NAVIGATION_ACTION_AGENT_DEBUG){Serial.println("AVOIDING N");}
            // Avoid the north entity
            configNorthEntity();

        } else if (this->state->northWestEntity != FLAT) {
            if(DEBUG && NAVIGATION_ACTION_AGENT_DEBUG){Serial.println("AVOIDING NW");}
            // Avoid the north west entity
            configNorthWestEntity();

        } else if (this->state->northEastEntity != FLAT) {
            if(DEBUG && NAVIGATION_ACTION_AGENT_DEBUG){Serial.println("AVOIDING NE");}
            // Avoid the north east entity
            configNorthEastEntity();

        } else if (this->state->lineFollowingTable == COMPLETED && this->state->incline == COMPLETED && this->state->circleDirection != UNKNOWN) {
            if(DEBUG && NAVIGATION_ACTION_AGENT_DEBUG){Serial.println("GOING TO CIRCLE");}
            // Go to circle
            configCircleDirection();

        } else if (nextManoeuver != NO_MANOEUVER){
            if(DEBUG && NAVIGATION_ACTION_AGENT_DEBUG){Serial.println("PERFORMING MANOEUVER");}
            // Perform manoeuver
            configManoeuver();

        } else if (this->state->westEntity != FLAT) {
            if(DEBUG && NAVIGATION_ACTION_AGENT_DEBUG){Serial.println("AVOIDING W");}
            // Avoid the west entity
            configWestEntity();

        }   else if (this->state->lineFollowingTable == CURRENT) {
            if(DEBUG && NAVIGATION_ACTION_AGENT_DEBUG){Serial.println("FOLLOWING LINE");}
            // Follow line
            configLineFollowing();

        } else if (this->state->ticTacState == CURRENT) {
            if(DEBUG && NAVIGATION_ACTION_AGENT_DEBUG){Serial.println("DROPPING TICTAC");}
            // Stop if dropping
            configTicTacDropping();
            
        } //else if (this->state->incline == CURRENT) {
//            if(DEBUG && NAVIGATION_ACTION_AGENT_DEBUG){Serial.println("CLIMBING");}
//            // Go faster up
//            configIncline();
//
//        } else if (this->state->decline == CURRENT) {
//            if(DEBUG && NAVIGATION_ACTION_AGENT_DEBUG){Serial.println("DECLINING");}
//            // Go slower down
//            configDecline();

        else {
            if(DEBUG && NAVIGATION_ACTION_AGENT_DEBUG){Serial.println("DEFAULT");}
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

/********************
 * Configuration of the navigation for each different state of the robot
*********************/

void NavigationActionAgent::configCircleDirection() {
    if (this->state->circleDirection == WEST) {
        turnLeftSpot();
    } else if(this->state->circleDirection == SOUTHWEST) {
        turnLeftSpot();
    } else if(this->state->circleDirection == SOUTH) {
        goReverse();
    } else if(this->state->circleDirection == SOUTHEAST) {
        turnRightSpot();
    } else if(this->state->circleDirection == EAST) {
        turnRightSpot();
    } else if(this->state->circleDirection == NORTHEAST) {
        turnRightSpot();
    } else if(this->state->circleDirection == NORTH) {
        goStraight();
    } else if(this->state->circleDirection == NORTHWEST) {
        turnLeftSpot();
    } else {
        configureDefault(); //DISCUSS: what should happen at this point? stopping is probably not a good idea
    }
}

void NavigationActionAgent::configLineFollowing() {
    if(this->state->lineState == LEFT) {
        turnLeftSpot();
    } else if(this->state->lineState == RIGHT) {
        turnRightSpot();
    } else if(this->state->lineState == CENTER) {
        goStraight();
    } else {
        goReverse(); //DISCUSS: what should happen at this point? stopping is probably not a good idea (maybe set max speed lower afterwards?)
    }
}

void NavigationActionAgent::configTicTacDropping() {
    stopMoving();
}

void NavigationActionAgent::configNorthWestEntity() {
    if(state->northWestEntity == EDGE) {
        if(DEBUG && NAVIGATION_ACTION_AGENT_DEBUG){Serial.println("EDGE DETECTED");}
    } else if(state->northWestEntity == OBSTACLE)  {
        if(DEBUG && NAVIGATION_ACTION_AGENT_DEBUG){Serial.println("OBSTACLE DETECTED");}
    }
    goReverse();
    this->nextManoeuver = TURN_60_DEGREE_RIGHT;
}

void NavigationActionAgent::configNorthEntity() {
    if(state->northEntity == EDGE) {
        if(DEBUG && NAVIGATION_ACTION_AGENT_DEBUG){Serial.println("EDGE DETECTED");}
    } else if(state->northEntity == OBSTACLE)  {
        if(DEBUG && NAVIGATION_ACTION_AGENT_DEBUG){Serial.println("OBSTACLE DETECTED");}
    }
    goReverse();
    this->nextManoeuver = TURN_90_DEGREE_RIGHT;
}

void NavigationActionAgent::configNorthEastEntity() {
    if(state->northEastEntity == EDGE) {
        if(DEBUG && NAVIGATION_ACTION_AGENT_DEBUG){Serial.println("EDGE DETECTED");}
    } else if(state->northEastEntity == OBSTACLE)  {
        if(DEBUG && NAVIGATION_ACTION_AGENT_DEBUG){Serial.println("OBSTACLE DETECTED");}
    }
    goReverse();
    this->nextManoeuver = TURN_120_DEGREE_RIGHT;
}

void NavigationActionAgent::configWestEntity() {
    goStraightRight();
}

void NavigationActionAgent::configIncline() {
    int speed = ROBOT_SPEED * RATIO_INCLINE;
    goStraight(speed);
}

void NavigationActionAgent::configDecline() {
    int speed = ROBOT_SPEED * RATIO_DECLINE;
    goStraight(speed);
}

void NavigationActionAgent::configManoeuver() {
    if (this->nextManoeuver == TURN_60_DEGREE_RIGHT) {
        performTurnXDegreeRightManoeuver(ROBOT_SPEED *TURN_90_DEGREE_RIGHT_MANOEUVER_TIME / 15 * 0.8);
    } else if (this->nextManoeuver == TURN_90_DEGREE_RIGHT) {
        performTurnXDegreeRightManoeuver(ROBOT_SPEED *TURN_90_DEGREE_RIGHT_MANOEUVER_TIME / 15);
    } else if (this->nextManoeuver == TURN_120_DEGREE_RIGHT) {
        performTurnXDegreeRightManoeuver(ROBOT_SPEED *TURN_90_DEGREE_RIGHT_MANOEUVER_TIME / 15 * 1.33);
    }
}

void NavigationActionAgent::configureDefault() {
    goStraightLeft();
}

/********************
 * Manoeuvers
*********************/

void NavigationActionAgent::performTurnXDegreeRightManoeuver(unsigned long manoeuverAmountTime) {
    if(this->manoeuverStartTime == 0) {
        this->manoeuverStartTime = millis();
    }

    if(millis() - this->manoeuverStartTime < manoeuverAmountTime) {
        turnRightSpot();
    } else {
        this->manoeuverStartTime = 0;
        this->nextManoeuver = NO_MANOEUVER;
    }
}

/********************
 * Basic navigation direction functions
*********************/
void NavigationActionAgent::goStraight(int speed) {
    if(DEBUG && NAVIGATION_ACTION_AGENT_DEBUG){Serial.println("GOING STRAIGHT");}
    // Go straight forward
    this->leftSpeed = speed; 
    this->rightSpeed = speed; 
    this->leftForward = true;
    this->rightForward = true;
}

void NavigationActionAgent::goReverse() {
    if(DEBUG && NAVIGATION_ACTION_AGENT_DEBUG){Serial.println("GOING REVERSE");}
    // Go straight backwards
    this->leftSpeed = ROBOT_SPEED; 
    this->rightSpeed = ROBOT_SPEED; 
    this->leftForward = false;
    this->rightForward = false;
}

void NavigationActionAgent::goStraightLeft() {
    if(DEBUG && NAVIGATION_ACTION_AGENT_DEBUG){Serial.println("GOING STRAIGHT LEFT");}
    // Go straight forward but deviate to the left
    this->leftSpeed = ROBOT_SPEED / RATIO_FAST_TO_SLOW_MOTOR;
    this->rightSpeed = ROBOT_SPEED * RATIO_FAST_TO_SLOW_MOTOR;
    this->leftForward = true;
    this->rightForward = true;
}

void NavigationActionAgent::turnLeft() {
    if(DEBUG && NAVIGATION_ACTION_AGENT_DEBUG){Serial.println("TURN LEFT");}
    // Turn the robot left
    this->leftSpeed = 0; 
    this->rightSpeed = ROBOT_SPEED; 
    this->leftForward = true;
    this->rightForward = true;
}

void NavigationActionAgent::turnLeftSpot() {
    if(DEBUG && NAVIGATION_ACTION_AGENT_DEBUG){Serial.println("TURN LEFT SPOT");}
    // Turn the robot left on the spot
    this->leftSpeed = ROBOT_SPEED * 1.2;
    this->rightSpeed = ROBOT_SPEED * 1.2 * 1.3;
    this->leftForward = false;
    this->rightForward = true;
}

void NavigationActionAgent::goStraightRight() {
    if(DEBUG && NAVIGATION_ACTION_AGENT_DEBUG){Serial.println("GOING STRAIGHT RIGHT");}
    // Go straight forward but deviate to the left
    this->leftSpeed = ROBOT_SPEED / RATIO_FAST_TO_SLOW_MOTOR; 
    this->rightSpeed = ROBOT_SPEED * RATIO_FAST_TO_SLOW_MOTOR;
    this->leftForward = true;
    this->rightForward = true;
}

void NavigationActionAgent::turnRight() {
    if(DEBUG && NAVIGATION_ACTION_AGENT_DEBUG){Serial.println("TURN RIGHT");}
    // Turn the robot right
    this->leftSpeed = ROBOT_SPEED; 
    this->rightSpeed = 0; 
    this->leftForward = true;
    this->rightForward = true;
}

void NavigationActionAgent::turnRightSpot() {
    if(DEBUG && NAVIGATION_ACTION_AGENT_DEBUG){Serial.println("TURN RIGHT SPOT");}
    // Turn the robot right on the spot
    this->leftSpeed = ROBOT_SPEED * 1.2 * 1.3;
    this->rightSpeed = ROBOT_SPEED * 1.2;
    this->leftForward = true;
    this->rightForward = false;
}

void NavigationActionAgent::stopMoving() {
    if(DEBUG && NAVIGATION_ACTION_AGENT_DEBUG){Serial.println("STOP MOVING");}
    // Stop the robot
    this->leftSpeed = 0; 
    this->rightSpeed = 0; 
    this->leftForward = true;
    this->rightForward = true;
}

