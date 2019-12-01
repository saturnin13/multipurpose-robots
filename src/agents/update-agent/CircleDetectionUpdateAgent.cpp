#include <Arduino.h>
#include "CircleDetectionUpdateAgent.hpp"

CircleDetectionUpdateAgent::CircleDetectionUpdateAgent(State* state, IRSensor* irNW, IRSensor* irNE, IRSensor* irSW, IRSensor* irSE, LineFollowerSensor* lf)
: UpdateAgent(state), lf(lf), irNW(irNW), irNE(irNE), irSW(irSW), irSE(irSE) {

}

void CircleDetectionUpdateAgent::update() {
    if(DEBUG && CIRCLE_DETECTION_UPDATE_AGENT_DEBUG){Serial.print("\nCircleDetectionUpdateAgent: ");}
//DISCUSS: maybe one full turn before and then start doing this? -> finalTable == CURRENT

    //if we have not completed the line following, then we cannot be close to the circle
    if(this->state->lineFollowingTable != COMPLETED || this->state->robotState == DISARMED) {
        if(DEBUG && CIRCLE_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("LINE FOLLOWING NOT COMPLETED OR ROBOT DISARMED, RETURNING");}
return;
    }

    //get sensor values
    bool nw = !irNW->lineDetected;
    bool ne = !irNE->lineDetected;
    bool sw = !irSW->lineDetected;
    bool se = !irSE->lineDetected;
    bool n = lf->unanimousDetection();
    unsigned int now = millis();

    //if there is an edge on the left, the IR sensors on the left are not useful as they might detect not existing lines
    if(this->state->northWestEntity == EDGE) {
        nw = false;
    }
    
    if(this->state->westEntity == EDGE) {
        sw = false;
    }

    //if there is an edge in the front, the IR sensors on the left are not useful as they might detect not existing lines
    if(this->state->northEastEntity == EDGE) {
        ne = false;
    }

    if(this->state->northEastEntity == EDGE && this->state->northWestEntity) {
        n = false;
    }

    if(this->state->northWestEntity == EDGE) {
        nw = false;
    }

    //if every sensor detects a black surface, we are in the circle and have therefore the final table
    if(nw && n && ne && sw && se) {
        //TODO: start timer here and stop after some time passed (stopping in the middle of the table
        //maybe make timer for time on circle (drive forward) and after leaving the circle reverse for half the time?

        //this means we finished the last table, so we are done
        if(DEBUG && CIRCLE_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("IN CIRLCE :)");}
this->state->finalTable = COMPLETED;
        
    //the circle is in the corresponding direction if the opposing IR Sensors do not detect values
    } else if(nw && n && ne && !sw && !se) {
        if(DEBUG && CIRCLE_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("CIRCLE IS N");}
this->state->circleDirection = NORTH;
        
    } else if(!nw && !n && !ne && sw && se) {
        if(DEBUG && CIRCLE_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("CIRCLE IS S");}
this->state->circleDirection = SOUTH;
        
    } else if(nw && !n && !ne && sw && !se) {
        if(DEBUG && CIRCLE_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("CIRCLE IS W");}
this->state->circleDirection = WEST;
        
    } else if(!nw && !n && ne && !sw && se) {
        if(DEBUG && CIRCLE_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("CIRCLE IS E");}
this->state->circleDirection = EAST;
        
    } else if(nw && (!n || n) && !ne && !sw && !se) {
        if(DEBUG && CIRCLE_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("CIRCLE IS NW");}
this->state->circleDirection = NORTHWEST;
        
    } else if(!nw && (!n || n) && ne && !sw && !se) {
        if(DEBUG && CIRCLE_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("CIRCLE IS NE");}
this->state->circleDirection = NORTHEAST;
        
    } else if(!nw && !n && !ne && sw && !se) {
        if(DEBUG && CIRCLE_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("CIRCLE IS SW");}
this->state->circleDirection = SOUTHWEST;
        
    } else if(!nw && !n && !ne && !sw && se) {
        if(DEBUG && CIRCLE_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("CIRCLE IS SE");}
this->state->circleDirection = SOUTHEAST;
        
    } else {
        if(DEBUG && CIRCLE_DETECTION_UPDATE_AGENT_DEBUG){Serial.println("CIRCLE LOCATION IS UNKNOWN");}
this->state->circleDirection = UNKNOWN;
    }
}