#include "StateMachine.hpp"

StateMachine::StateMachine() {
    state.LeftEntity = FLAT;
    state.FrontEntity = FLAT;

    state.LineFollowingTable = UNSEEN;
    state.HillTable = UNSEEN
    state.FinalTable = UNSEEN;

    state.totalTurnedDegrees = 0;

    state.emergencyStop = false;
}