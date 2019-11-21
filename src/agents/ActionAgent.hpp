#ifndef ACTIONAGENT_H
#define ACTIONAGENT_H

#include "agent.hpp"

class ActionAgent : public Agent {
public:
    // Perform the action of the actuator.
    virtual void enact() = 0;
};

#endif