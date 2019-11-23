#ifndef ACTIONAGENT_H
#define ACTIONAGENT_H

#include "../Agent.hpp"
#include "../../state/State.hpp"

class ActionAgent : public Agent {
public:
    ActionAgent(State* state);
    // Perform the action of the actuator.
    virtual void enact() = 0;
};

#endif