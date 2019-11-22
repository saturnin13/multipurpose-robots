#ifndef MULTIPURPOSE_ROBOTS_LEDACTIONAGENT_H
#define MULTIPURPOSE_ROBOTS_LEDACTIONAGENT_H

#include "../../actuators/LED.hpp"
#include "ActionAgent.hpp"
#include "../../state_machine/State.hpp"

class LEDActionAgent : public ActionAgent {
public:
    LEDActionAgent(State* state, LED* led);

    void enact();
    
private:
    LED* led;
};

#endif
