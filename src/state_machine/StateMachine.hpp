#ifndef STATE_H
#define STATE_H

enum TableState {UNSEEN, CURRENT, COMPLETED};
enum Entity {FLAT, EDGE, OBSTACLE};
enum RobotState {ARMED, DISARMED};
enum TicTacState {UNDROPPED, DROPPING, DROPPED};
enum LineOrientation {LOST, LEFT, CENTER, RIGHT};
enum CircleOrientation {UNKNOWN, WEST, SOUTHWEST, SOUTH, SOUTHEAST, EAST, NORTHEAST, NORTH, NORTHWEST};

struct State {
    RobotState robotState;

    Entity leftEntity;
    Entity frontEntity;

    TableState lineFollowingTable;
    TableState incline;
    TableState finalTable;
    
    TicTacState ticTacState;

    LineOrientation lineState;

    CircleOrientation circleOrientation;

    int totalTurnedDegrees;

    bool emergencyStop;

    long initTime;
};

class StateMachine {
public:
    StateMachine();
    // Relevant state parameters are included here
    State state;
};

#endif