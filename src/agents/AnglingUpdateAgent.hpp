#ifndef MULTIPURPOSE_ROBOTS_ANGLINGUPDATEAGENT_H
#define MULTIPURPOSE_ROBOTS_ANGLINGUPDATEAGENT_H

#include "../sensors/IMUSensor.hpp"
#include "UpdateAgent.hpp"
#include "../state_machine/StateMachine.hpp"

class AnglingUpdateAgent : public UpdateAgent {
public:
    // Update the state.
    AnglingUpdateAgent(State &state, IMUSensor imu);
    void update() = 0;

private:
    State state;
    IMUSensor imu;
};

#endif
