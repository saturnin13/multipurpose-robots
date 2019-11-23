#include "TicTacActionAgent.hpp"
#include "ActionAgent.hpp"

#define NUMBER_STEPS_PER_TICTAC 555
#define NUMBER_TICTACS 4

TicTacActionAgent::TicTacActionAgent(State* state, StepperMotor* stepperMotor, int numberDrops)
: ActionAgent(state), stepperMotor(stepperMotor), numberDrops(numberDrops) {
    this->stepsToStartPosition = 0; //assumes start state is all the way back
}

void TicTacActionAgent::enact() {
    this->stepperMotor->configure(true, 2, 610); //ca 3600 is one full turn

    /*if ((this->state->robotState == DISARMED) && (this->state->finalTable != COMPLETED)) {
        this->led->configure(SOLID_RED);
    } else if(this->state->robotState == ARMED) {
        this->led->configure(STROBE_GREEN);
    } else if ((this->state->robotState == DISARMED) && (this->state->finalTable == COMPLETED)) {
        this->led->configure(FLASH_BLUE);
    }*/

        if((millis() - initTime) > 1000) {
        stepperMotor.enact();
        if(!stepperMotor.workInProgress && numberTicTacs>0) {
            numberTicTacs--;
            delay(1000);
            stepperMotor.configure(true, 2, (555)); //510 + numberTicTacs*10
        } else if (numberTicTacs == 1) {
            stepperMotor.configure(false, 2, 2200); //ca 3600 is one full turn
            numberTicTacs = 0;
        }
    }

    //TODO: needs to know amount of tic tacs, constant for way forward, add up way backwards, bool workInProgress
    //specify number of tic tacs, say if go back or not


    //led->enact();
}