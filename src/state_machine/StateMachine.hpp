#ifndef STATE_H
#define STATE_H

enum TableState {UNSEEN, CURRENT, COMPLETED};
enum Entity {FLAT, EDGE, OBSTACLE};
enum RobotState {ARMED, DISARMED};

struct State {
    RobotState robotState = DISARMED;

    Entity leftEntity = FLAT;
    Entity frontEntity = FLAT;

    TableState lineFollowingTable = UNSEEN;
    TableState incline = UNSEEN;
    TableState finalTable = UNSEEN;

    bool ticTacDropped = false;

    int totalTurnedDegrees = 0;

    bool emergencyStop = false;
};

class StateMachine {
public:
    StateMachine();
    // Relevant state parameters are included here
    State state;
};

#endif