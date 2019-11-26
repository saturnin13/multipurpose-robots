#ifndef UPDATEAGENT_H
#define UPDATEAGENT_H

#include "../../state/State.hpp"
#include "../Agent.hpp"

class UpdateAgent : public Agent {
  public:
    UpdateAgent(State *state);
    // Update the state.
    virtual void update() = 0;
};

#endif