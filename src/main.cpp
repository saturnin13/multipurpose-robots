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
 * Update agents
*********************/
AnglingUpdateAgent angleAgent(&state, &imu);
ButtonUpdateAgent buttonAgent(&state, &button);
CircleDetectionUpdateAgent circleAgent(&state, &irFrontLeft, &irFrontRight, &irRearLeft, &irRearRight, &lf);
EntityDetectionUpdateAgent entityAgent(&state, &lf, &usRearLeft, &usDownLeft, &usFrontLeft, &usFrontMiddle, &usFrontRight, &usDownRight);
LineDetectionUpdateAgent lineAgent(&state, &lf);

/********************
 * Action agents
*********************/
LEDActionAgent ledAgent(&state, &led);
TicTacActionAgent ticTacAgent(&state, &stepperMotor, 4);
NavigationActionAgent navigationAgent(&state, &leftMotor, &rightMotor);

/********************
 * ROS
 *********************/
#if ROS
void eStopCallback(const std_msgs::Bool &msg) {
    state.emergencyStop = msg.data;
}

void dropCallback(const std_msgs::Empty &msg) {
    if (state.ticTacState == UNDROPPED) {
        state.ticTacState = REQUESTED;
    }
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Bool> estopSubscriber(ROS_ESTOP_TOPIC, &eStopCallback);
ros::Subscriber<std_msgs::Empty> dropSubscriber(ROS_DROP_TOPIC, &dropCallback);
#endif


void setup() {
  
    Wire.endTransmission(true); //otherwise it doesnt work
    Serial.begin(9600);
    //todo count time since initialization and wait x sec
    state.setupTime = millis();


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

    //usFrontLeft.update();
    //usFrontMiddle.update();
    //usFrontRight.update();
    usDownLeft.update();
    usDownRight.update();
    usRearLeft.update();

    //irFrontLeft.update();
    //irFrontRight.update();
    //irRearLeft.update();
    //irRearRight.update();

    lf.update();

    // 2. Let update agents compute state
    //angleAgent.update();
    buttonAgent.update();
    //circleAgent.update();
    entityAgent.update();
    //lineAgent.update();
    //ticTacUpdateAgent.update(dropRequested);
    //eStopAgent.update(eStopRequested);
    
    // 3. Make action agents carry out actions
    ledAgent.enact();

    //wait for some time until all sensors are initialized and stuff, move this to state agent and make sure
    //navigation, tic tac and arming will not be done
    if(millis() - state.setupTime < START_DELAY_TIME) {
        return;
    }
    
    //ticTacAgent.enact();
    navigationAgent.enact();

    //TODO test button insted of this below this comment
    if(button.pressed) {
        state.robotState = ARMED;
    }
    if(button.pressed && state.emergencyStop) {
        state.emergencyStop = false;
    }

    /* TODO List:
    DISCUSS: LineDetectionAgent l33
    DISCUSS: difference between armed (LED) and move (3 sec in the beginning): new state
    DISUCSS: i think we need to drastically improve the
            edge detection (on the hardware side: MDF for positioning of US, try IR as backup sensor) and
            the navigation (one the software side): reverse back from edge and turn then

    UpdateAgents:
    IDEA: StateUpdateAgent, makes sure robot is armed at right point, sets time to now,
        handles everything the others should not do +  stuff from button agent + reset &init function for state
        TODO: now as part of state

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

    if(DEBUG) {
    //state.emergencyStop = false;
    //Serial.println(lf.lineDetected2);
    //delay(100);

    //Serial.print("State front: ");Serial.println(state.northEntity);
    //Serial.print("State front,left: ");Serial.println(state.northWestEntity);
    //Serial.print("State front,right: ");Serial.println(state.northEastEntity);
    //Serial.print("State rear,left: ");Serial.println(state.southWestEntity);
    //Serial.print("INCLINE ");Serial.println(state.incline);
    //Serial.print("DROPPING ");Serial.println(state.ticTacState);
    //Serial.print("Button: ");Serial.println(button.pressed);
    //Serial.print("State ");Serial.println(state.robotState);

    //Serial.println(state.incline);
    //Serial.print(usFrontLeft.distance);Serial.print(" , ");
    //Serial.print(usFrontMiddle.distance);Serial.print(" , ");
    //Serial.print(usFrontRight.distance);Serial.print(" , ");

    Serial.print(usRearLeft.distance);Serial.print(" - ");
    Serial.print(usDownLeft.distance);Serial.print(" , ");
    Serial.print(usDownRight.distance);Serial.print(" , ");
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

