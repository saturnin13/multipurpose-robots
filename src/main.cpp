#include <Arduino.h>
#include <Wire.h>
#include <ros.h>
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
#include "agents/update-agent/TicTacUpdateAgent.hpp"
#include "agents/update-agent/EStopUpdateAgent.hpp"

#include "agents/action-agents/LEDActionAgent.hpp"
#include "agents/action-agents/TicTacActionAgent.hpp"
#include "agents/action-agents/NavigationActionAgent.hpp"

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
 * ROS
*********************/
bool dropRequested = false;
bool eStopRequested = false;
void messageDrop( const std_msgs::Empty& toggle_msg){
    dropRequested = true;
}
void messageEStop( const std_msgs::Empty& toggle_msg){
    eStopRequested = true;
}
ros::Subscriber<std_msgs::Empty> subTT("drop", &messageDrop );
ros::Subscriber<std_msgs::Empty> subES("estop", &messageEStop );
ros::NodeHandle nh;

/********************
 * Update agents
*********************/
AnglingUpdateAgent angleAgent(&state, &imu);
ButtonUpdateAgent buttonAgent(&state, &button);
CircleDetectionUpdateAgent circleAgent(&state, &irFrontLeft, &irFrontRight, &irRearLeft, &irRearRight, &lf);
EntityDetectionUpdateAgent entityAgent(&state, &usFrontLeft, &usFrontRight, &usFrontMiddle, &usDownLeft, &usDownRight, &usRearLeft);
LineDetectionUpdateAgent lineAgent(&state, &lf);
TicTacUpdateAgent ticTacUpdateAgent(&state, &nh, subTT);
EStopUpdateAgent eStopAgent(&state, &nh, subES);

/********************
 * Action agents
*********************/
LEDActionAgent ledAgent(&state, &led);
TicTacActionAgent ticTacAgent(&state, &stepperMotor);
NavigationActionAgent navigationAgent(&state, &leftMotor, &rightMotor);


void setup() {
  
    Wire.endTransmission(true); //otherwise it doesnt work
    Serial.begin(9600);

    if(DEBUG) {
        Serial.println("Setup done!");
    }
    
}

void loop() {

    // 1. Update sensors
    button.update();
    imu.update();

    usFrontLeft.update();
    usFrontMiddle.update();
    usFrontRight.update();
    usDownLeft.update();
    usDownRight.update();
    usRearLeft.update();

    irFrontLeft.update();
    irFrontRight.update();
    irRearLeft.update();
    irRearRight.update();

    lf.update();

    // 2. Let update agents compute state
    //angleAgent.update();
    //buttonAgent.update();
    //circleAgent.update();
    entityAgent.update();
    lineAgent.update();
    //ticTacUpdateAgent.update(dropRequested);
    //eStopAgent.update(eStopRequested);
    
    // 3. Make action agents carry out actions
    //ledAgent.enact();
    //ticTacAgent.enact();
    state.robotState = ARMED;
    navigationAgent.enact();

    Serial.println(leftMotor.stopped);

    /* TODO List:
    DISCUSS: LineDetectionAgent l33

    UpdateAgents:
        DONE -> (EdgeDetection, ObstacleDetection) -> EntityDetection
        DONE -> LineDetection
        DONE -> CircleDetection
        DONE -> LoopDetection
        DONE -> E-stopRequest (-> ROS)
        DONE -> TicTacUpdateAgent: TicTacDropRequest (->ROS) + Update TicTacState (Dropping, Dropped):   
    Optional -> add IR sensors for edge detection (backup)

    ActionAgents:
        DONE -> TicTacDropper +  reload mode
        DONE -> Driver (Navigation,     - initial search direction (i.e. which line to follow first)) + return if disarmed
        DONE -> Led Agent
    Optional -> display agent
    TODO: ROS debugging agent, send state.toString all x seconds or if something changes

    Actuators:
        DONE -> stepper motor
        DONE -> motor
    Optional -> display

    Sensor:
    TODO: Error correction: weighted average values over several measurements, make sure no negative distances occur
   
    TODO: Update everything at once
    */


    if(DEBUG) {
    Serial.println(state.northEntity);

    //Serial.println(state.incline);
    //Serial.print(usFrontLeft.distance);Serial.print(" , ");
    //Serial.print(usFrontMiddle.distance);Serial.print(" , ");
    //Serial.print(usFrontRight.distance);Serial.print(" , ");
    //Serial.print(usDownLeft.distance);Serial.print(" , ");
    //Serial.print(usDownRight.distance);Serial.print(" , ");
    //Serial.print(usRearLeft.distance);Serial.print(" - ");
    //Serial.print(ir1.value);Serial.print(" , ");
    //Serial.print(ir2.value);Serial.print(" , ");
    //Serial.print(ir3.value);Serial.print(" , ");
    //Serial.print(ir4.value);Serial.print(" , ");
    //Serial.print(lf.lineDetected0);Serial.print(" , ");
    //Serial.print(lf.lineDetected1);Serial.print(" , ");
    //Serial.print(lf.lineDetected2);Serial.print(" , ");
    //Serial.print(lf.lineDetected3);Serial.print(" , ");
    //Serial.print(lf.lineDetected4);Serial.print(" , ");
    //Serial.print(imu.xAngle);Serial.print(" , ");
    //Serial.print(imu.yAngle);Serial.print(" , ");
    //Serial.println(b1.pressed);
    Serial.println(' ');
    //delay(50);
    }

}

