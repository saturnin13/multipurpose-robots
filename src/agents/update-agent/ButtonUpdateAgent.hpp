#ifndef MULTIPURPOSE_ROBOTS_BUTTONUPDATEAGENT_H
#define MULTIPURPOSE_ROBOTS_BUTTONUPDATEAGENT_H

#include "../../sensors/ButtonSensor.hpp"
#include "UpdateAgent.hpp"
#include "../../state/State.hpp"
#include "../../Constants.hpp"
#include <Arduino.h>


class ButtonUpdateAgent : public UpdateAgent {
public:
    ButtonUpdateAgent(State* state, ButtonSensor* b);

    void update();

private:
    ButtonSensor* b;
};
#endif
