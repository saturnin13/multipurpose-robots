#ifndef ACTIONAGENT_H
#define ACTIONAGENT_H

class ActionAgent : public Agent{
public:
    // Perform the action of the actuator.
    virtual void enact() = 0;
};

#endif