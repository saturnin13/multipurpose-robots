#ifndef AGENT_H
#define AGENT_H

class ActionAgent : public Agent{
public:
    // Perform the action of the actuator.
    virtual void enact() = 0;
};

#endif