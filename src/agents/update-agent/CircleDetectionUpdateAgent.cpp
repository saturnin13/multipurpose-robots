#include <Arduino.h>
#include "CircleDetectionUpdateAgent.hpp"


CircleDetectionUpdateAgent::CircleDetectionUpdateAgent(State* state, IRSensor* irNW, IRSensor* irNE, IRSensor* irSW, IRSensor* irSE, LineFollowerSensor* lf)
: UpdateAgent(state), irNW(irNW), irNE(irNE), irSW(irSW), irSE(irSE), lf(lf) {

}

void CircleDetectionUpdateAgent::update() {

    //TODO: maybe one full turn before and then start doing this? -> finalTable == CURRENT

    //if we have not completed the line following, then we cannot be close to the circle
    if(this->state->lineFollowingTable != COMPLETED) {
        return;
    }

    //get sensor values
    bool nw = irNW->obstacleDetected;
    bool ne = irNE->obstacleDetected;
    bool sw = irSW->obstacleDetected;
    bool se = irSE->obstacleDetected;
    bool n = lf->unanimousDetection;

    //if there is an edge on the left, the IR sensors on the left are not useful as they might detect not existing lines
    if(this->state->leftEntity == EDGE) {
        nw = false;
        sw = false;
    }

    //if there is an edge in the front, the IR sensors on the left are not useful as they might detect not existing lines
    if(this->state->frontEntity == EDGE) {
        n = false;
        nw = false;
        ne = false;
    }

    //if every sensor detects a black surface, we are in the circle and have therefore the final table
    if(nw && n && ne && sw && se) {
        this->state->finalTable = COMPLETED;
    //the circle is in the corresponding direction if the opposing IR Sensors do not detect values
    } else if(nw && n && ne && !sw && !se) {
        this->state->circleOrientation = NORTH;
    } else if(!nw && !n && !ne && sw && se) {
        this->state->circleOrientation = SOUTH;
    } else if(nw && !n && !ne && sw && !se) {
        this->state->circleOrientation = WEST;
    } else if(!nw && !n && ne && !sw && se) {
        this->state->circleOrientation = EAST;
    } else if(nw && (!n || n) && !ne && !sw && !se) {
        this->state->circleOrientation = NORTHWEST;
    } else if(!nw && (!n || n) && ne && !sw && !se) {
        this->state->circleOrientation = NORTHEAST;
    } else if(!nw && !n && !ne && sw && !se) {
        this->state->circleOrientation = SOUTHWEST;
    } else if(!nw && !n && !ne && !sw && se) {
        this->state->circleOrientation = SOUTHEAST;
    } else {
        this->state->circleOrientation = UNKNOWN;
    }
}