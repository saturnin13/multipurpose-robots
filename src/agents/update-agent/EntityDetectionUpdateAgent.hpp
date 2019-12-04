#ifndef MULTIPURPOSE_ROBOTS_ENTITYDETECTIONUPDATEAGENT_H
#define MULTIPURPOSE_ROBOTS_ENTITYDETECTIONUPDATEAGENT_H

#include "../../sensors/IRSensor.hpp"
#include "../../sensors/LineFollowerSensor.hpp"
#include "../../sensors/UltrasonicSensor.hpp"
#include "UpdateAgent.hpp"
#include "../../state/State.hpp"
#include "../../Constants.hpp"

class EntityDetectionUpdateAgent : public UpdateAgent {
public:
    EntityDetectionUpdateAgent(State* state, UltrasonicSensor* usSW, UltrasonicSensor* usNW, UltrasonicSensor* usWForward,
    UltrasonicSensor* usNForward, UltrasonicSensor* usNE);

    void update();

private:
    LineFollowerSensor* lf;
    UltrasonicSensor* usSW;
    UltrasonicSensor* usNW;
    UltrasonicSensor* usWForward;
    UltrasonicSensor* usNForward;
    UltrasonicSensor* usNE;

};

#endif
