#include "CircleDetectionUpdateAgent.hpp"
#include <Arduino.h>

CircleDetectionUpdateAgent::CircleDetectionUpdateAgent(State* state, IRSensor* irNW, IRSensor* irNE, IRSensor* irSW, IRSensor* irSE)
: UpdateAgent(state), irNW(irNW), irNE(irNE), irSW(irSW), irSE(irSE) {

}

void CircleDetectionUpdateAgent::update() {
    if (DEBUG && CIRCLE_DETECTION_UPDATE_AGENT_DEBUG) {
        Serial.print("\nCircleDetectionUpdateAgent: ");
    }
    // DISCUSS: maybe one full turn before and then start doing this? ->
    // finalTable == CURRENT

    //if we have not completed the line following, then we cannot be close to the circle
    if(this->state->lineFollowingTable != COMPLETED || this->state->incline != COMPLETED || this->state->robotState == DISARMED) {
        if(DEBUG && CIRCLE_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("LINE FOLLOWING NOT COMPLETED OR ROBOT DISARMED, RETURNING");}
        return;
    }

    // get sensor values
    bool nw = !irNW->lineDetected;
    bool ne = !irNE->lineDetected;
    bool sw = !irSW->lineDetected;
    bool se = !irSE->lineDetected;

    // if there is an edges on the left, the IR sensors on the left are not
    // useful as they might detect not existing lines
    if (this->state->northWestEntity != FLAT) {
        nw = false;
    }

    if (this->state->northEastEntity != FLAT) {
        ne = false;
    }

    if (this->state->northEntity != FLAT) {
        nw = false;
        ne = false;
    }

    if (this->state->westEntity != FLAT) {
        nw = false;
        sw = false;
    }

    //if every sensor detects a black surface, we are in the circle and have therefore the final table
    if(nw && ne && sw && se) {
        //maybe make timer for time on circle (drive forward) and after leaving the circle reverse for half the time?

        //this means we finished the last table, so we are done
        if(DEBUG && CIRCLE_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("IN CIRLCE :)");}
        this->state->finalTable = COMPLETED;
        this->state->robotState = DISARMED;

        // the circle is in the corresponding direction if the opposing IR
        // Sensors do not detect values
    } else if((nw && ne && sw && !se) || (nw && !ne && !sw && !se)) {
        if(DEBUG && CIRCLE_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("CIRCLE IS NW");}
        this->state->circleDirection = NORTHWEST;
        
    } else if((nw && ne && !sw && se) || (!nw && ne && !sw && !se)) {
        if(DEBUG && CIRCLE_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("CIRCLE IS NE");}
        this->state->circleDirection = NORTHEAST;
        
    } else if((nw && !ne && sw && se) || (!nw && !ne && sw && !se)) {
        if(DEBUG && CIRCLE_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("CIRCLE IS SW");}
        this->state->circleDirection = SOUTHWEST;
        
    } else if((!nw && ne && sw && se) || (!nw && !ne && !sw && se)) {
        if(DEBUG && CIRCLE_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("CIRCLE IS SE");}
        this->state->circleDirection = SOUTHEAST;
        
    } else if(nw && ne && !sw && !se) {
        if(DEBUG && CIRCLE_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("CIRCLE IS N");}
        this->state->circleDirection = NORTH;
        
    } else if(nw && !ne && sw && !se) {
        if(DEBUG && CIRCLE_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("CIRCLE IS W");}
        this->state->circleDirection = WEST;
        
    } else if(!nw && ne && !sw && se) {
        if(DEBUG && CIRCLE_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("CIRCLE IS E");}
        this->state->circleDirection = EAST;
        
    } else if(!nw && !ne && sw && se) {
        if(DEBUG && CIRCLE_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("CIRCLE IS S");}
        this->state->circleDirection = SOUTH;
        
    } else {
        if(DEBUG && CIRCLE_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("CIRCLE LOCATION IS UNKNOWN");}
        this->state->circleDirection = UNKNOWN;
    }
}