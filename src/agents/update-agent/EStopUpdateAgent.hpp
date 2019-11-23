#ifndef MULTIPURPOSE_ROBOTS_ESTOPUPDATEAGENT_H
#define MULTIPURPOSE_ROBOTS_ESTOPNUPDATEAGENT_H

//#include "../../sensors/ButtonSensor.hpp"
#include "UpdateAgent.hpp"
#include "../../state/State.hpp"
#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Empty.h>

class EStopUpdateAgent : public UpdateAgent {
public:
    EStopUpdateAgent(State* state, ros::NodeHandle* nh, ros::Subscriber<std_msgs::Empty> subES);
    
    void update();

    void update(bool eStopRequested);

private:
    State* state;
    ros::NodeHandle* nh;
    ros::Subscriber<std_msgs::Empty> subES;
    //ButtonSensor* b;

    bool eStopRequested;

};

#endif