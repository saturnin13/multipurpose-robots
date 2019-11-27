#include <Arduino.h>
#include "CircleDetectionUpdateAgent.hpp"

#define PRINTING_CIRCLE_ORIENTATION 0

CircleDetectionUpdateAgent::CircleDetectionUpdateAgent(State* state, IRSensor* irNW, IRSensor* irNE, IRSensor* irSW, IRSensor* irSE, LineFollowerSensor* lf)
: UpdateAgent(state), lf(lf), irNW(irNW), irNE(irNE), irSW(irSW), irSE(irSE) {

}

void CircleDetectionUpdateAgent::update() {

    //DISCUSS: maybe one full turn before and then start doing this? -> finalTable == CURRENT

    //if we have not completed the line following, then we cannot be close to the circle
    if(this->state->lineFollowingTable != COMPLETED || this->state->robotState == DISARMED) {
        return;
    }

    //Serial.print("IR: ");Serial.println(irSW->lineDetected);

    //get sensor values
    bool nw = !irNW->lineDetected;
    bool ne = !irNE->lineDetected;
    bool sw = !irSW->lineDetected;
    bool se = !irSE->lineDetected;
    bool n = lf->unanimousDetection();
    unsigned int now = millis();

    if(now % 1000 > 1) {Serial.print(nw);Serial.print(n);Serial.print(ne);Serial.print(sw);Serial.println(se);}
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
        //DISCUSS: start timer here and stop after some time passed (stopping in the middle of the table

        //this means we finished the last table, so we are done
        this->state->finalTable = COMPLETED;

        if(PRINTING_CIRCLE_ORIENTATION) {Serial.println("IN CIRLCE :)");}

    //the circle is in the corresponding direction if the opposing IR Sensors do not detect values
    } else if(nw && n && ne && !sw && !se) {
        if(PRINTING_CIRCLE_ORIENTATION) {Serial.println("CIRCLE IS N");}
        this->state->circleDirection = NORTH;
    } else if(!nw && !n && !ne && sw && se) {
        if(PRINTING_CIRCLE_ORIENTATION) {Serial.println("CIRCLE IS S");}
        this->state->circleDirection = SOUTH;
    } else if(nw && !n && !ne && sw && !se) {
        if(PRINTING_CIRCLE_ORIENTATION) {Serial.println("CIRCLE IS W");}
        this->state->circleDirection = WEST;
    } else if(!nw && !n && ne && !sw && se) {
        if(PRINTING_CIRCLE_ORIENTATION) {Serial.println("CIRCLE IS E");}
        this->state->circleDirection = EAST;
    } else if(nw && (!n || n) && !ne && !sw && !se) {
        if(PRINTING_CIRCLE_ORIENTATION) {Serial.println("CIRCLE IS NW");}
        this->state->circleDirection = NORTHWEST;
    } else if(!nw && (!n || n) && ne && !sw && !se) {
        if(PRINTING_CIRCLE_ORIENTATION) {Serial.println("CIRCLE IS NE");}
        this->state->circleDirection = NORTHEAST;
    } else if(!nw && !n && !ne && sw && !se) {
        if(PRINTING_CIRCLE_ORIENTATION) {Serial.println("CIRCLE IS SW");}
        this->state->circleDirection = SOUTHWEST;
    } else if(!nw && !n && !ne && !sw && se) {
        if(PRINTING_CIRCLE_ORIENTATION) {Serial.println("CIRCLE IS SE");}
        this->state->circleDirection = SOUTHEAST;
    } else {
        if(PRINTING_CIRCLE_ORIENTATION) {Serial.println("CIRCLE IS UNKNOWN");}
        this->state->circleDirection = UNKNOWN;
    }
}