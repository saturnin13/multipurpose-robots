#include "TicTacActionAgent.hpp"
#include "ActionAgent.hpp"
#include <Arduino.h>

#define NUMBER_STEPS_FIRST_TICTAC 610
#define NUMBER_STEPS_PER_TICTAC 555
#define NUMBER_TICTACS 4
#define TIME_BETWEEN_DROPS 1000
#define DELTA_STEPS_BACKWARDS 100

TicTacActionAgent::TicTacActionAgent(State* state, StepperMotor* stepperMotor, int numberDrops)
: ActionAgent(state), stepperMotor(stepperMotor), numberDrops(numberDrops) {
    this->stepsToStartPosition = 0; //assumes start state is all the way back
    this->numberTicTacs = NUMBER_TICTACS;
    this -> timeLastDropEnd = 0;
}

void TicTacActionAgent::enact() {

    //this agent is only active if we are armed
    if(this->state->robotState != ARMED)
    {
        return;
    }

    //this agent is only active if we want to drop something
    if(this->state->ticTacState != DROPPING)
    {
        return;
    }

    //we just change something in the stepper motor if it is not working
    if(!this->stepperMotor->workInProgress) {
        unsigned long now = millis();

        //save time since the last drop was ended
        if(timeLastDropEnd == 0) {
            timeLastDropEnd = millis();
        }

        //drive stepper motor outwards if we have drops left and we waited some time after last drop
        if(numberDrops > 0 && (timeLastDropEnd - now) > TIME_BETWEEN_DROPS) {

            //first drop has more step (making sure the tictac is acatually dropped)
            int numberSteps = numberTicTacs == 4 ?  NUMBER_STEPS_FIRST_TICTAC : NUMBER_STEPS_PER_TICTAC;
            
            //drop it
            this->stepperMotor->configure(true, 2, numberSteps); //ca 3600 is one full turn

            //save how many steps we made
            this->stepsToStartPosition += numberSteps;

            //decrement number of tic tacs to drop
            this->numberTicTacs--;
        
        //drive stepper motors back in if we do not have drops left and we waited some time after last drop
        } else if (numberDrops == 0 && (timeLastDropEnd - now) > TIME_BETWEEN_DROPS) {

            //get it back all the way
            this->stepperMotor->configure(false, 2, this->stepsToStartPosition-DELTA_STEPS_BACKWARDS); //ca 3600 is one full turn
            
            //make sure we stop doing anything from now on
            numberDrops = -1;
        }
    }
    
    //enact stepper motor after all
    stepperMotor->enact();

}