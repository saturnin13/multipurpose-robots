#ifndef MULTIPURPOSE_ROBOTS_LINEDETECTIONUPDATEAGENT_H
#define MULTIPURPOSE_ROBOTS_LINEDETECTIONUPDATEAGENT_H

#include "../../sensors/IRSensor.hpp"
#include "../../sensors/LineFollowerSensor.hpp"
#include "../../sensors/UltrasonicSensor.hpp"
#include "UpdateAgent.hpp"
#include "../../state/State.hpp"
#include "../../Constants.hpp"

class LineDetectionUpdateAgent : public UpdateAgent {
public:
    LineDetectionUpdateAgent(State* state, LineFollowerSensor* lf);

    void update();

private:
    LineFollowerSensor* lf;
};

#endif
