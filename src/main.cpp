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

#include "agents/update-agent/AnglingUpdateAgent.hpp"
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


/* TODO List:
    TODO: if not armed, dont change states, otherwise we will detect inclines before...
    DISCUSS: LineDetectionAgent l33
    DISCUSS: difference between armed (LED) and move (3 sec in the beginning): new state
    DISUCSS: i think we need to drastically improve the
            edge detection (on the hardware side: MDF for positioning of US, try IR as backup sensor) and
            the navigation (one the software side): reverse back from edge and turn then
    TODO: integrate ros stuff

    UpdateAgents:
    IDEA: StateUpdateAgent, makes sure robot is armed at right point, sets time to now,
        handles everything the others should not do +  stuff from button agent + reset &init function for state

        DONE -> (EdgeDetection, ObstacleDetection) -> EntityDetection
        DONE -> LineDetection
        DONE -> CircleDetection
        DONE -> LoopDetection
        DOING: E-stopRequest (-> ROS)
    TicTacUpdateAgent: TicTacDropRequest (->ROS) + Update TicTacState (Dropping, Dropped):
        //make constant for time to wait for drop
        //let tictacupdateagent time that and set tictacs then to dropped
        DOING -> TicTacUpdateAgent: TicTacDropRequest (->ROS) + Update TicTacState (Dropping, Dropped):
    OPTIONAL -> add IR sensors for edge detection (backup)

    ActionAgents:
    TODO: ROS debugging agent, send state.toString all x seconds or if something changes

        DONE -> TicTacDropper +  reload mode
        DONE -> Driver (Navigation,     - initial search direction (i.e. which line to follow first)) + return if disarmed
        DONE -> Led Agent
    Optional -> display agent
    ROS debugging agent, send state.toString all x seconds or if something changes


    Actuators:
        DONE -> stepper motor
        DONE -> motor
    OPTIONAL -> display

    Sensor:

    TODO: Error correction: weighted average values over several measurements, make sure no negative distances occur
    DISCUSS: problem with detection speed for edges (if we average over several values, new things need time to take effect?)

    OPTIONAL: Update everything at once
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
UltrasonicSensor usNForward(US_N_FORWARD_TRIGGER_PIN, US_N_FORWARD_ECHO_PIN);
UltrasonicSensor usNNWDown(US_NNW_DOWN_TRIGGER_PIN, US_NNW_DOWN_ECHO_PIN);
UltrasonicSensor usNNEDown(US_NNE_DOWN_TRIGGER_PIN, US_NNE_DOWN_ECHO_PIN);
UltrasonicSensor usNWDown(US_SW_DOWN_TRIGGER_PIN, US_SW_DOWN_ECHO_PIN);

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
AnglingUpdateAgent anglingUpdateAgent(&state, &imu);
ButtonUpdateAgent buttonUpdateAgent(&state, &button);
CircleDetectionUpdateAgent circleDetectionUpdateAgent(&state, &irNW, &irNE, &irNW, &irSE, &lf);
EntityDetectionUpdateAgent entityDetectionUpdateAgent(&state, &lf, &usNWForward, &usNNWDown, &usNWForward, &usNForward, &usNEForward, &usNNEDown);
LineDetectionUpdateAgent lineDetectionUpdateAgent(&state, &lf, &usNWForward, &usNEForward);
LoopDetectionUpdateAgent loopDetectionUpdateAgent(&state, &imu);
TicTacUpdateAgent ticTacUpdateAgent(&state);

/********************
 * Action agents
*********************/
LEDActionAgent ledAgent(&state, &led);
NavigationActionAgent navigationAgent(&state, &leftMotor, &rightMotor);
TicTacActionAgent ticTacAgent(&state, &stepperMotor, 4);

/********************
 * ROS
 *********************/
#if ROS
void eStopCallback(const std_msgs::Bool &msg) {
    state.emergencyStop = msg.data;
    digitalWrite(13, !digitalRead(13));
}

void dropCallback(const std_msgs::Empty &msg) {
    digitalWrite(13, !digitalRead(13));
    if (state.ticTacState == UNSEEN) {
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
    irNW.update();
    irSE.update();

    lf.update();

    usNWForward.update();
    usNForward.update();
    usNEForward.update();
    usNNWDown.update();
    usNNEDown.update();
    usNWForward.update();
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

    //wait for some time until all sensors are initialized and stuff, move this to state agent and make sure
    //navigation, tic tac and arming will not be done
    if(millis() - state.setupTime < START_DELAY_TIME) {
        return;
    }

    ticTacAgent.enact();
    navigationAgent.enact();
}

void printDebug() {
    /********************
     * State Printing
    *********************/
    Serial.print("The current robot state is: {");

    Serial.print("robotState: ");Serial.print(state.robotState);

    Serial.print("southWestEntity: ");Serial.print(state.southWestEntity);
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

    Serial.print(", initializationTime: ");Serial.print(state.initializationTime);
    Serial.print(", setupTime: ");Serial.print(state.setupTime);

    Serial.println("}");

    /********************
     * Other
    *********************/

    //Serial.print(usNWForward.distance);Serial.print(" , ");
    //Serial.print(usNForward.distance);Serial.print(" , ");
    //Serial.print(usNEForward.distance);Serial.print(" , ");

    //Serial.print(usNWForward.distance);Serial.print(" - ");
    //Serial.print(usNNWDown.distance);Serial.print(" , ");
    //Serial.print(usNNEDown.distance);Serial.print(" , ");
    //Serial.print(ir1.value);Serial.print(" , ");
    //Serial.print(ir2.value);Serial.print(" , ");
    //Serial.print(ir3.value);Serial.print(" , ");
    //Serial.print(ir4.value);Serial.print(" , ");
    //Serial.print(lf.values[0]);Serial.print(" , ");
    //Serial.print(lf.values[1]);Serial.print(" , ");
    //Serial.print(lf.values[2]);Serial.print(" , ");
    //Serial.print(lf.values[3]);Serial.print(" , ");
    //Serial.print(lf.values[4]);Serial.print(" , ");
    //Serial.print(imu.xAngle);Serial.print(" , ");
    //Serial.print(imu.yAngle);Serial.print(" , ");
    //Serial.println(b1.pressed);

    Serial.println("");
}

void setup() {
  
    Wire.endTransmission(true); //otherwise it doesnt work
    //Serial.begin(9600);
    Serial.begin(115200);
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    state.robotState = DISARMED;
    state.setupTime = millis();
    //state.robotState = DISARMED;
    

    if(DEBUG) {
        Serial.println("Setup done");
    }

    nh.subscribe(estopSubscriber);
    nh.subscribe(dropSubscriber);

}

void loop() {
    //Serial.println("LOOPing");
    #if ROS
    nh.spinOnce();
    #endif

    // 1. Update sensors
    updateSensors();

    // 2. Let update agents compute state
    updateAgents();
    
    // 3. Make action agents carry out actions
    enactAgents();

    // TODO: test button instead of this below this comment
    /*if(button.pressed) {
        state.robotState = ARMED;
    }*/
    /*if(button.pressed && state.emergencyStop) {
        state.emergencyStop = false;
    }*/
    // TODO: STATEUPDATEAGENT
    if(state.emergencyStop) {
        state.robotState = DISARMED;
    }

    if(DEBUG) {
        printDebug();
    }

    delay(MAIN_LOOP_DELAY);
}