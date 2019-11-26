#ifndef MULTIPURPOSE_ROBOTS_ANGLINGUPDATEAGENT_H
#define MULTIPURPOSE_ROBOTS_ANGLINGUPDATEAGENT_H

#include "../../Constants.hpp"
#include "../../sensors/IMUSensor.hpp"
#include "../../state/State.hpp"
#include "UpdateAgent.hpp"

class AnglingUpdateAgent : public UpdateAgent {
  public:
    AnglingUpdateAgent(State *state, IMUSensor *imu);

    void update();

  private:
    IMUSensor *imu;
};

#endif
