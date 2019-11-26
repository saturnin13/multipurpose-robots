#ifndef INCLINEUPDATEAGENT_H
#define INCLINEUPDATEAGENT_H

#include "../../Constants.hpp"
#include "../../sensors/IMUSensor.hpp"
#include "../../state/State.hpp"
#include "UpdateAgent.hpp"

class InclineUpdateAgent : public UpdateAgent {
  public:
    InclineUpdateAgent(State *state, IMUSensor *imu);

    void update();

  private:
    IMUSensor *imu;
};

#endif
