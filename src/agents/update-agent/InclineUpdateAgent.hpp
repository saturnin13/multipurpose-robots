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

    // The time at which the incline threshold was first exceeded.
    unsigned long thresholdExceededTime = 0;

    // Whether an incline is being detected right now. Used internally for debouncing.
    bool inclineDetected = false;
};

#endif
