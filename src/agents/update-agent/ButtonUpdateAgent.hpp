#ifndef MULTIPURPOSE_ROBOTS_BUTTONUPDATEAGENT_H
#define MULTIPURPOSE_ROBOTS_BUTTONUPDATEAGENT_H

#include "../../sensors/ButtonSensor.hpp"
#include "UpdateAgent.hpp"
#include "../../state_machine/State.hpp"

class ButtonUpdateAgent : public UpdateAgent {
public:
    ButtonUpdateAgent(State* state, ButtonSensor* b);

    void update();

private:
    State* state;
    ButtonSensor* b;
};

#endif
