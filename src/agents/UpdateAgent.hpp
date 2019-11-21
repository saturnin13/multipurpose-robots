#ifndef UPDATEAGENT_H
#define UPDATEAGENT_H

#include "Agent.hpp"

class UpdateAgent : public Agent {
public:
    // Update the state.
    virtual void update() = 0;
};

#endif