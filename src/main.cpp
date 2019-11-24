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

#include "agents/action-agents/LEDActionAgent.hpp"
#include "agents/action-agents/TicTacActionAgent.hpp"

#include "state/State.hpp"

#include "Constants.hpp"

/********************
 * State
*********************/
State state;

/********************
 * Sensors
*********************/
UltrasonicSensor usFrontLeft(US_FRONT_LEFT_TRIGGER_PIN, US_FRONT_LEFT_ECHO_PIN);
UltrasonicSensor usFrontRight(US_FRONT_RIGHT_TRIGGER_PIN, US_FRONT_RIGHT_ECHO_PIN);
UltrasonicSensor usFrontMiddle(US_FRONT_MIDDLE_TRIGGER_PIN, US_FRONT_MIDDLE_ECHO_PIN);
UltrasonicSensor usDownLeft(US_DOWN_LEFT_TRIGGER_PIN, US_DOWN_LEFT_ECHO_PIN);
UltrasonicSensor usDownRight(US_DOWN_RIGHT_TRIGGER_PIN, US_DOWN_RIGHT_ECHO_PIN);
UltrasonicSensor usRearLeft(US_REAR_LEFT_TRIGGER_PIN, US_REAR_LEFT_ECHO_PIN);
IMUSensor imu;
IRSensor irFrontLeft(IR_FRONT_LEFT_PIN);
IRSensor irFrontRight(IR_FRONT_RIGHT_PIN);
IRSensor irRearLeft(IR_REAR_LEFT_PIN);
IRSensor irRearRight(IR_REAR_RIGHT_PIN);
ButtonSensor button(START_BUTTON_PIN);
LineFollowerSensor lf(LINE_FOLLOWER_PIN0, LINE_FOLLOWER_PIN1, LINE_FOLLOWER_PIN2, LINE_FOLLOWER_PIN3, LINE_FOLLOWER_PIN4);

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
AnglingUpdateAgent angleAgent(&state, &imu);
ButtonUpdateAgent buttonAgent(&state, &button);
CircleDetectionUpdateAgent circleAgent(&state, &irFrontLeft, &irFrontRight, &irRearLeft, &irRearRight, &lf);
EntityDetectionUpdateAgent entityAgent(&state, &usFrontLeft, &usFrontRight, &usFrontMiddle, &usDownLeft, &usDownRight, &usRearLeft);
LineDetectionUpdateAgent lineAgent(&state, &lf);

/********************
 * Action agents
*********************/
LEDActionAgent ledAgent(&state, &led);
TicTacActionAgent ticTacAgent(&state, &stepperMotor);

/********************
 * ROS
 *********************/
#if ROS
void eStopCallback(const std_msgs::Bool &msg) {
    state.emergencyStop = msg.data;
}

void dropCallback(const std_msgs::Empty &msg) {
    // TODO: Set that drop command has been issued.
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Bool> estopSubscriber(ROS_ESTOP_TOPIC, &eStopCallback);
ros::Subscriber<std_msgs::Empty> dropSubscriber(ROS_DROP_TOPIC, &dropCallback);
#endif


void setup() {
  
    Wire.endTransmission(true); //otherwise it doesnt work
    Serial.begin(9600);

    if(DEBUG) {
        Serial.println("Setup done");
    }

    nh.subscribe(estopSubscriber);
    nh.subscribe(dropSubscriber);

}

void loop() {

    #if ROS
    nh.spinOnce();
    #endif

    // 1. Update sensors
    button.update();
    imu.update();

    // usFrontLeft.update();
    // usFrontMiddle.update();
    // usFrontRight.update();
    // usDownLeft.update();
    // usDownRight.update();
    // usBackLeft.update();
    // ir1.update();
    // ir2.update();
    // ir3.update();
    // ir4.update();
    // lf.update();

    // 2. Let update agents compute state
    angleAgent.update();
    buttonAgent.update();
    
    // 3. Make action agents carry out actions
    ledAgent.enact();
    ticTacAgent.enact();

    /* TODO List:
    UpdateAgents:
        DONE -> (EdgeDetection, ObstacleDetection) -> EntityDetection
        DONE -> LineDetection
        DONE -> CircleDetection
        DONE -> LoopDetection
    E-stopRequest (-> ROS)
    TicTacUpdateAgent: TicTacDropRequest (->ROS) + Update TicTacState (Dropping, Dropped):   
        //make constant for time to wait for drop
        //let tictacupdateagent time that and set tictacs then to dropped
    Optional -> add IR sensors for edge detection (backup)

    ActionAgents:
        DONE -> TicTacDropper +  reload mode
        DONE -> Driver (Navigation,     - initial search direction (i.e. which line to follow first)) + return if disarmed
        DONE -> Led Agent
    Optional -> display agent
    ROS debugging agent, send state.toString all x seconds or if something changes

    Actuators:
        DONE -> stepper motor
        DONE -> motor
    Optional -> display

    Sensor:
    Error correction: weighted average values over several measurements, make sure no negative distances,... occur
    */
    

    if(DEBUG) {
    //Serial.println(state.incline);
    //Serial.print(usFrontLeft.distance);Serial.print(" , ");
    //Serial.print(usFrontMiddle.distance);Serial.print(" , ");
    //Serial.print(usFrontRight.distance);Serial.print(" , ");
    //Serial.print(usDownLeft.distance);Serial.print(" , ");
    //Serial.print(usDownRight.distance);Serial.print(" , ");
    //Serial.print(usBackLeft.distance);Serial.print(" - ");
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
    //Serial.println(' ');
    delay(100);
    }

}

