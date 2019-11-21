#include "StateMachine.hpp"

StateMachine::StateMachine() {
    state.robotState = DISARMED;
    state.leftEntity = FLAT;
    state.frontEntity = FLAT;

    state.lineFollowingTable = UNSEEN;
    state.incline = UNSEEN;
    state.finalTable = UNSEEN;

    state.ticTacDropped = false;

    state.totalTurnedDegrees = 0;

    state.emergencyStop = false;

    state.initTime = 0;

}