#ifndef AGENT_H
#define AGENT_H

#include "../state_machine/StateMachine.hpp"

class Agent {
public:
    Agent(State* state);
protected:
    State* state;
};

#endif