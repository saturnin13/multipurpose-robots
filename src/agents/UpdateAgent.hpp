#ifndef AGENT_H
#define AGENT_H

class UpdateAgent : public Agent {
public:
    // Perform the action of the actuator.
    virtual void update() = 0;
};

#endif