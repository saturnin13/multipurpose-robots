#ifndef MULTIPURPOSE_ROBOTS_CIRCLEDETECTIONUPDATEAGENT_H
#define MULTIPURPOSE_ROBOTS_CIRCLEDETECTIONUPDATEAGENT_H

#include "../../sensors/IRSensor.hpp"
#include "../../sensors/LineFollowerSensor.hpp"
#include "../../state/State.hpp"
#include "UpdateAgent.hpp"
#include "../../Constants.hpp"

class CircleDetectionUpdateAgent : public UpdateAgent {
  public:
    CircleDetectionUpdateAgent(State *state, IRSensor *irNW, IRSensor *irNE, IRSensor *irSW, IRSensor *irSE);

    void update();

  private:
    IRSensor *irNW;
    IRSensor *irNE;
    IRSensor *irSW;
    IRSensor *irSE;
};

#endif
