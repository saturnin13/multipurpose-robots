#include <Arduino.h>
#include "CircleDetectionUpdateAgent.hpp"


CircleDetectionUpdateAgent::CircleDetectionUpdateAgent(State* state, IRSensor* irNW, IRSensor* irNE, IRSensor* irSW, IRSensor* irSE, LineFollowerSensor* lf)
: UpdateAgent(state), lf(lf), irNW(irNW), irNE(irNE), irSW(irSW), irSE(irSE) {

}

void CircleDetectionUpdateAgent::update() {

    //TODO: maybe one full turn before and then start doing this? -> finalTable == CURRENT

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
    //Serial.println("DETECTING CIRLCES...");
    //if every sensor detects a black surface, we are in the circle and have therefore the final table
    if(nw && n && ne && sw && se) {
        //this means we finished the last table, so we are done
        this->state->finalTable = COMPLETED;
        Serial.println("IN CIRLCES :)");
        //TODO: double check or move
        this->state->robotState = DISARMED;
    //the circle is in the corresponding direction if the opposing IR Sensors do not detect values
    } else if(nw && n && ne && !sw && !se) {
        Serial.println("N");
        this->state->circleDirection = NORTH;
    } else if(!nw && !n && !ne && sw && se) {
        Serial.println("S");
        this->state->circleDirection = SOUTH;
    } else if(nw && !n && !ne && sw && !se) {
        Serial.println("WEST");
        this->state->circleDirection = WEST;
    } else if(!nw && !n && ne && !sw && se) {
        Serial.println("East");
        this->state->circleDirection = EAST;
    } else if(nw && (!n || n) && !ne && !sw && !se) {
        Serial.println("NW");
        this->state->circleDirection = NORTHWEST;
    } else if(!nw && (!n || n) && ne && !sw && !se) {
        Serial.println("NE");
        this->state->circleDirection = NORTHEAST;
    } else if(!nw && !n && !ne && sw && !se) {
        this->state->circleDirection = SOUTHWEST;
        Serial.println("SW");
    } else if(!nw && !n && !ne && !sw && se) {
        this->state->circleDirection = SOUTHEAST;
        Serial.println("SE");

    } else {
        this->state->circleDirection = UNKNOWN;
        Serial.println("UNKNOWN");

    }
}