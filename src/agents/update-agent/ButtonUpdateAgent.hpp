#ifndef MULTIPURPOSE_ROBOTS_BUTTONUPDATEAGENT_H
#define MULTIPURPOSE_ROBOTS_BUTTONUPDATEAGENT_H

#include "../../Constants.hpp"
#include "../../sensors/ButtonSensor.hpp"
#include "../../state/State.hpp"
#include "UpdateAgent.hpp"
#include <Arduino.h>

class ButtonUpdateAgent : public UpdateAgent {
  public:
    ButtonUpdateAgent(State *state, ButtonSensor *button);

    void update();

  private:
    ButtonSensor *button;
    bool lastButtonState;
    unsigned long timeSinceArming;
};
#endif
