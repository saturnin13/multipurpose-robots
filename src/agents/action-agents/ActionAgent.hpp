#ifndef ACTIONAGENT_H
#define ACTIONAGENT_H

#include "../../state/State.hpp"
#include "../Agent.hpp"
#include "../../Constants.hpp"

class ActionAgent : public Agent {
  public:
    ActionAgent(State *state);
    // Perform the action of the actuator.
    virtual void enact() = 0;
};

#endif