#ifndef MULTIPURPOSE_ROBOTS_TICTACUPDATEAGENT_H
#define MULTIPURPOSE_ROBOTS_TICTACUPDATEAGENT_H

#include "../../state/State.hpp"
#include "UpdateAgent.hpp"
#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Empty.h>

class TicTacUpdateAgent : public UpdateAgent {
  public:
    TicTacUpdateAgent(State *state);

    void update();

  private:
    State *state;
    unsigned long timeDropStart;
};

#endif