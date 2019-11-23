#ifndef MULTIPURPOSE_ROBOTS_TICTACUPDATEAGENT_H
#define MULTIPURPOSE_ROBOTS_TICTACUPDATEAGENT_H

#include "UpdateAgent.hpp"
#include "../../state/State.hpp"
#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Empty.h>

class TicTacUpdateAgent : public UpdateAgent {
public:
    TicTacUpdateAgent(State* state, ros::NodeHandle* nh, ros::Subscriber<std_msgs::Empty> subTT);
    
    void update();
    
    void update(bool dropRequested);

private:
    State* state;
    ros::NodeHandle* nh;
    ros::Subscriber<std_msgs::Empty> subTT;

    bool dropRequested;
    unsigned long timeDropStart;
};

#endif