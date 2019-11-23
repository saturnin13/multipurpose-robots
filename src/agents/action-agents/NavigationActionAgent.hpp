#ifndef MULTIPURPOSE_ROBOTS_NAVIGATIONACTIONAGENT_H
#define MULTIPURPOSE_ROBOTS_NAVIGATIONACTIONAGENT_H

#include "../../actuators/Motor.hpp"
#include "ActionAgent.hpp"
#include "../../state/State.hpp"

enum Priority {PRIO1, PRIO2, PRIO3};

class NavigationActionAgent : public ActionAgent {
public:
    NavigationActionAgent(State* state, Motor* leftMotor, Motor* rightMotor);

    void enact();
    
private:
    Motor* leftMotor;
    Motor* rightMotor;

    int leftSpeed; 
    int rightSpeed; 
    bool leftForward;
    bool rightForward;

    Priority priority;

};

#endif
