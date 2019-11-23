#ifndef MULTIPURPOSE_ROBOTS_TICTACACTIONAGENT_H
#define MULTIPURPOSE_ROBOTS_TICTACACTIONAGENT_H

#include "../actuators/StepperMotor.hpp"
#include "ActionAgent.hpp"
#include "../state_machine/StateMachine.hpp"

class TicTacActionAgent : public ActionAgent {
public:
    TicTacActionAgent(State* state, StepperMotor* stepperMotor, int numberDrops = 1);

    void enact();
    
private:
    StepperMotor* stepperMotor;
    int numberDrops;
    int stepsToStartPosition;
};

#endif
