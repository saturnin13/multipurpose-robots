#ifndef MULTIPURPOSE_ROBOTS_NAVIGATIONACTIONAGENT_H
#define MULTIPURPOSE_ROBOTS_NAVIGATIONACTIONAGENT_H

#include "../../Constants.hpp"
#include "../../actuators/Motor.hpp"
#include "../../state/State.hpp"
#include "ActionAgent.hpp"

class NavigationActionAgent : public ActionAgent {
  public:
    NavigationActionAgent(State *state, Motor *leftMotor, Motor *rightMotor);

    void enact();

  private:
    Motor *leftMotor;
    Motor *rightMotor;

    // Robot speed and orientation
    int leftSpeed;
    int rightSpeed;
    bool leftForward;
    bool rightForward;

    // Robot speed and orientation configuration for each task
    void configCircleDirection();
    void configLineFollowing();
    void configTicTacDropping();
    void configNorthWestEntity();
    void configNorthEntity();
    void configNorthEastEntity();
    void configWestEntity();
    void configureDefault();

    // Basic robot command
    void turnLeft();
    void goStraightLeft();
    void turnLeftSpot();
    void goStraight();
    void goStraightRight();
    void turnRightSpot();
    void turnRight();
    void stopMoving();
    void goReverse();
};

#endif
