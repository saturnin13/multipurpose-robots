#include <Arduino.h>
#include <Wire.h>
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

#include "actuators/LED.hpp"
#include "actuators/StepperMotor.hpp"
#include "actuators/Motor.hpp"

#include "sensors/ButtonSensor.hpp"
#include "sensors/IMUSensor.hpp"
#include "sensors/IRSensor.hpp"
#include "sensors/LineFollowerSensor.hpp"
#include "sensors/UltrasonicSensor.hpp"

#include "agents/update-agent/InclineUpdateAgent.hpp"
#include "agents/update-agent/ButtonUpdateAgent.hpp"
#include "agents/update-agent/CircleDetectionUpdateAgent.hpp"
#include "agents/update-agent/EntityDetectionUpdateAgent.hpp"
#include "agents/update-agent/LineDetectionUpdateAgent.hpp"
#include "agents/update-agent/LoopDetectionUpdateAgent.hpp"
#include "agents/update-agent/TicTacUpdateAgent.hpp"

#include "agents/action-agents/LEDActionAgent.hpp"
#include "agents/action-agents/NavigationActionAgent.hpp"
#include "agents/action-agents/TicTacActionAgent.hpp"

#include "state/State.hpp"

#include "Constants.hpp"


/* List of things we need to consider:

    DISUCSS: i think we need to drastically improve the
            edge detection (on the hardware side: MDF for positioning of US, try IR as backup sensor) and
            the navigation (one the software side): reverse back from edge and turn then

    Misc:
        DONE -> if not armed, dont change states, otherwise we will detect inclines before...
        DONE -> LineDetectionAgent l33
        DONE -> difference between armed (LED) and move (3 sec in the beginning): new state

    UpdateAgents:
    IDEA -> StateUpdateAgent, makes sure robot is armed at right point, stuff from button agent + reset &init function for state
    OPTIONAL -> add IR sensors for edge detection (backup)

        DONE -> (EdgeDetection, ObstacleDetection) -> EntityDetection
        DONE -> LineDetection
        DONE -> CircleDetection
        DONE -> LoopDetection
        DONE -> E-stopRequest (-> ROS)
        DONE -> TicTacUpdateAgent: TicTacDropRequest (->ROS) + Update TicTacState (Dropping, Dropped):

    ActionAgents:
    TODO -> ROS debugging agent, send state.toString all x seconds or if something changes
    OPTIONAL -> display agent

        DONE -> TicTacDropper +  reload mode
        DONE -> Driver (Navigation,     - initial search direction (i.e. which line to follow first)) + return if disarmed
        DONE -> Led Agent

    Actuators:
    OPTIONAL -> display

        DONE -> stepper motor
        DONE -> motor

    Sensor:
    DISCUSS -> problem with detection speed for edges (if we average over several values, new things need time to take effect?)
    OPTIONAL -> Update everything at once
    */

/********************
 * State
*********************/
State state;

/********************
 * Sensors
*********************/
ButtonSensor button(START_BUTTON_PIN);
IMUSensor imu;
IRSensor irNW(IR_NW_PIN);
IRSensor irNE(IR_NE_PIN);
IRSensor irSW(IR_SW_PIN);
IRSensor irSE(IR_SE_PIN);
LineFollowerSensor lf(LINE_FOLLOWER_PIN0, LINE_FOLLOWER_PIN1, LINE_FOLLOWER_PIN2, LINE_FOLLOWER_PIN3, LINE_FOLLOWER_PIN4);
UltrasonicSensor usNWForward(US_NW_FORWARD_TRIGGER_PIN, US_NW_FORWARD_ECHO_PIN);
UltrasonicSensor usNEForward(US_NE_FORWARD_TRIGGER_PIN, US_NE_FORWARD_ECHO_PIN);
UltrasonicSensor usWForward(US_W_FORWARD_TRIGGER_PIN, US_W_FORWARD_ECHO_PIN);
UltrasonicSensor usNWDown(US_NW_DOWN_TRIGGER_PIN, US_NW_DOWN_ECHO_PIN);
UltrasonicSensor usNEDown(US_NE_DOWN_TRIGGER_PIN, US_NE_DOWN_ECHO_PIN);
UltrasonicSensor usSWDown(US_SW_DOWN_TRIGGER_PIN, US_SW_DOWN_ECHO_PIN);

/********************
 * Actuators
*********************/
LED led(LED_RED_PIN, LED_GREEN_PIN, LED_BLUE_PIN);
StepperMotor stepperMotor(STEPPER_MOTOR_PIN1, STEPPER_MOTOR_PIN2, STEPPER_MOTOR_PIN3, STEPPER_MOTOR_PIN4);
Motor leftMotor(MOTOR_PIN1, MOTOR_PIN2, MOTOR_ENA1);
Motor rightMotor(MOTOR_PIN3, MOTOR_PIN4, MOTOR_ENA2);

/********************
 * Update agents
*********************/
InclineUpdateAgent anglingUpdateAgent(&state, &imu);
ButtonUpdateAgent buttonUpdateAgent(&state, &button);
CircleDetectionUpdateAgent circleDetectionUpdateAgent(&state, &irNW, &irNE, &irSW, &irSE);

EntityDetectionUpdateAgent entityDetectionUpdateAgent(&state, &usSWDown, &usNWDown, &usNWForward, &usWForward, &usNEForward, &usNEDown);
LineDetectionUpdateAgent lineDetectionUpdateAgent(&state, &lf, &usNWDown, &usNEDown);

LoopDetectionUpdateAgent loopDetectionUpdateAgent(&state, &imu);
TicTacUpdateAgent ticTacUpdateAgent(&state);

/********************
 * Action agents
*********************/
LEDActionAgent ledAgent(&state, &led);
NavigationActionAgent navigationAgent(&state, &leftMotor, &rightMotor);
TicTacActionAgent ticTacAgent(&state, &stepperMotor, 1);

/********************
 * Debug variable
*********************/
unsigned long lastDebugUpdate = millis();

/********************
 * ROS
 *********************/
#if ROS
void eStopCallback(const std_msgs::Bool &msg) {
    bool stop = msg.data;

    if (stop) {
        state.emergencyStop = true;
        state.robotState = ARMED;
    } else {
        state.emergencyStop = false;
        state.robotState = DISARMED;
    }
}

void dropCallback(const std_msgs::Empty &msg) {
    if (state.robotState != DISARMED && state.ticTacState == UNSEEN) {
        state.ticTacState = CURRENT;
    }
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Bool> estopSubscriber(ROS_ESTOP_TOPIC, &eStopCallback);
ros::Subscriber<std_msgs::Empty> dropSubscriber(ROS_DROP_TOPIC, &dropCallback);
#endif

void updateSensors() {
    button.update();

    imu.update();

    irNW.update();
    irNE.update();
    irSW.update();
    irSE.update();

    lf.update();

    usNWForward.update();
    usWForward.update();
    usNEForward.update();
    usNWDown.update();
    usNEDown.update();
    usSWDown.update();
}

void updateAgents() {
    anglingUpdateAgent.update();
    buttonUpdateAgent.update();
    circleDetectionUpdateAgent.update();
    entityDetectionUpdateAgent.update();
    lineDetectionUpdateAgent.update();
    loopDetectionUpdateAgent.update();
    ticTacUpdateAgent.update();
}

void enactAgents() {
    ledAgent.enact();
    ticTacAgent.enact();
    navigationAgent.enact();
}

void printDebug() {
    auto now = millis();

    if (now < lastDebugUpdate + DEBUG_PRINTING_DELAY) {
        return;
    }

    lastDebugUpdate = now;

    Serial.print("The current robot state is: {");

    Serial.print("robotState: ");Serial.print(state.robotState);

    Serial.print(", westEntity: ");Serial.print(state.westEntity);
    Serial.print(", northEastEntity: ");Serial.print(state.northEastEntity);
    Serial.print(", northEntity: ");Serial.print(state.northEntity);
    Serial.print(", northWestEntity: ");Serial.print(state.northWestEntity);

    Serial.print(", incline: ");Serial.print(state.incline);
    Serial.print(", finalTable: ");Serial.print(state.finalTable);

    Serial.print(", ticTacState: ");Serial.print(state.ticTacState);

    Serial.print(", lineFollowingTable: ");Serial.print(state.lineFollowingTable);
    Serial.print(", lineState: ");Serial.print(state.lineState);

    Serial.print(", circleDirection: ");Serial.print(state.circleDirection);

    Serial.print(", totalTurnedQuadrants: ");Serial.print(state.totalTurnedQuadrants);

    Serial.print(", emergencyStop: ");Serial.print(state.emergencyStop);

    Serial.println("}");

    Serial.println("");
    Serial.println("");
}

void setup() {

    // Workaround for IMU
    Wire.endTransmission(true);
    
    Serial.begin(BAUD_RATE);

    if (DEBUG) {
        Serial.println("Setup completed");
    }
    state.incline = COMPLETED;
    state.decline = COMPLETED;
    state.ticTacState = COMPLETED;

    #if ROS
    nh.getHardware()->setBaud(BAUD_RATE);
    nh.initNode();
    nh.subscribe(estopSubscriber);
    nh.subscribe(dropSubscriber);
    #endif
}

void loop() {
    #if ROS
    nh.spinOnce();
    #endif

    // 1. Update sensors
    updateSensors();

    // 2. Let update agents compute state
    updateAgents();

    // 3. Make action agents carry out actions
    enactAgents();

    if (DEBUG) {
        printDebug();
    }

}
