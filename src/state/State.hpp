#ifndef STATE_H
#define STATE_H

enum RobotState { ARMED, DISARMED };
enum EventState { UNSEEN, CURRENT, COMPLETED };
enum Entity { FLAT, EDGE, OBSTACLE };
enum LineOrientation { LEFT, CENTER, RIGHT };
enum CircleDirection { UNKNOWN, WEST, SOUTHWEST, SOUTH, SOUTHEAST, EAST, NORTHEAST, NORTH, NORTHWEST };

struct State {
    RobotState robotState = DISARMED;

    Entity westEntity = FLAT;
    Entity northEastEntity = FLAT;
    Entity northEntity = FLAT;
    Entity northWestEntity = FLAT;

    EventState incline = UNSEEN;
    EventState decline = UNSEEN;
    EventState finalTable = UNSEEN;

    EventState ticTacState = UNSEEN;

    EventState lineFollowingTable = UNSEEN;
    LineOrientation lineState = CENTER; // Default initial value, only used while lineFollowingTable is CURRENT

    CircleDirection circleDirection = UNKNOWN;

    int totalTurnedQuadrants = 0;

    bool emergencyStop = false;

    bool move = false;
};

#endif