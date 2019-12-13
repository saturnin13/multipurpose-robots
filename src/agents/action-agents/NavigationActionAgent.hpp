#ifndef MULTIPURPOSE_ROBOTS_NAVIGATIONACTIONAGENT_H
#define MULTIPURPOSE_ROBOTS_NAVIGATIONACTIONAGENT_H

#include "../../Constants.hpp"
#include "../../actuators/Motor.hpp"
#include "../../state/State.hpp"
#include "ActionAgent.hpp"

#define MANOEUVERS
#define TURN_90_DEGREE_RIGHT_MANOEUVER_TIME 1000

enum Manoeuver { NO_MANOEUVER, TURN_120_DEGREE_RIGHT, TURN_90_DEGREE_RIGHT, TURN_60_DEGREE_RIGHT };

class NavigationActionAgent : public ActionAgent {
  public:
    NavigationActionAgent(State *state, Motor *leftMotor, Motor *rightMotor);

    void enact();

  private:
    Motor *leftMotor;
    Motor *rightMotor;

    // Manoeuver related
    Manoeuver nextManoeuver;
    unsigned long manoeuverStartTime;

    void performTurnXDegreeRightManoeuver(unsigned long manoeuverAmountTime);

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
    void configWestNorthWestEntity();
    void configWestEntity();
    void configIncline();
    void configDecline();
    void configManoeuver();
    void configureDefault();

    // Basic robot command
    void turnLeft();
    void goStraightLeft();
    void turnLeftSpot();
    void goStraight(int speed = ROBOT_SPEED);
    void goStraightRight();
    void turnRightSpot();
    void turnRight();
    void stopMoving();
    void goReverse();
};

#endif
