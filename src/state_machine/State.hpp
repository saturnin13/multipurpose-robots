#ifndef STATE_H
#define STATE_H

enum TableState { UNSEEN, CURRENT, COMPLETED };
enum Entity { FLAT, EDGE, OBSTACLE };
enum RobotState { ARMED, DISARMED };
enum TicTacState { UNDROPPED, DROPPING, DROPPED };
enum LineOrientation { LOST, LEFT, CENTER, RIGHT };
enum CircleOrientation { UNKNOWN, WEST, SOUTHWEST, SOUTH, SOUTHEAST, EAST, NORTHEAST, NORTH, NORTHWEST };

struct State {
    RobotState robotState = DISARMED;

    Entity leftEntity = FLAT;
    Entity frontEntity = FLAT;

    TableState lineFollowingTable = UNSEEN;
    TableState incline = UNSEEN;
    TableState finalTable = UNSEEN;
    
    TicTacState ticTacState = UNDROPPED;

    LineOrientation lineState = LOST;

    CircleOrientation circleOrientation = UNKNOWN;

    int totalTurnedDegrees = 0;

    bool emergencyStop = false;

    long initTime = 0;
};

#endif