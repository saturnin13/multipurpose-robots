#ifndef MULTIPURPOSE_ROBOTS_LOOPDETECTIONUPDATEAGENT_H
#define MULTIPURPOSE_ROBOTS_LOOPDETECTIONUPDATEAGENT_H

#include "../../sensors/IMUSensor.hpp"
#include "UpdateAgent.hpp"
#include "../../state/State.hpp"
#include "../../Constants.hpp"

class LoopDetectionUpdateAgent : public UpdateAgent {
public:
    LoopDetectionUpdateAgent(State* state, IMUSensor* imu);

    void update();

private:
    IMUSensor* imu;
    int quadrant;
    double previousQuadrant;
};

#endif
