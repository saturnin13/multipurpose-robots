#ifndef MULTIPURPOSE_ROBOTS_ANGLINGUPDATEAGENT_H
#define MULTIPURPOSE_ROBOTS_ANGLINGUPDATEAGENT_H

#include "../../sensors/IMUSensor.hpp"
#include "UpdateAgent.hpp"
#include "../../state/State.hpp"

class AnglingUpdateAgent : public UpdateAgent {
public:
    AnglingUpdateAgent(State* state, IMUSensor* imu);

    void update();

private:
    IMUSensor* imu;
};

#endif
