#ifndef UPDATEAGENT_H
#define UPDATEAGENT_H

#include "../Agent.hpp"
#include "../../state_machine/StateMachine.hpp"

class UpdateAgent : public Agent {
public:
    UpdateAgent(State* state);
    // Update the state.
    virtual void update() = 0;
};

#endif