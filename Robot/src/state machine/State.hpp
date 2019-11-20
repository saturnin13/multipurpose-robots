#ifndef STATE_H
#define STATE_H

class State {
public:
    State();
    // Relevant state parameters are included here
    enum Entity {FLAT, EDGE, OBSTACLE};
    Entity LeftParallelTo = FLAT;

    enum TableState {UNSEEN, CURRENT, COMPLETED};
    TableState LineFollowingTable = UNSEEN;
    TableState HillTable = UNSEEN
    TableState FinalTable = UNSEEN;
    bool totalTurnedDegrees = 0;
};

#endif