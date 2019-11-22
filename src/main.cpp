#include <Arduino.h>
#include <Wire.h>

#include "actuators/LED.hpp"

#include "sensors/ButtonSensor.hpp"
#include "sensors/IMUSensor.hpp"
#include "sensors/IRSensor.hpp"
#include "sensors/LineFollowerSensor.hpp"
#include "sensors/UltrasonicSensor.hpp"

#include "agents/action-agents/LEDActionAgent.hpp"
#include "agents/update-agent/AnglingUpdateAgent.hpp"
#include "agents/update-agent/ButtonUpdateAgent.hpp"

#include "state_machine/State.hpp"

#include "Constants.hpp"

/********************
 * State
*********************/
State state;

/********************
 * Sensors
*********************/
UltrasonicSensor usFrontLeft(46, 47);
UltrasonicSensor usFrontRight(44, 45);
UltrasonicSensor usFrontMiddle(42, 43);
UltrasonicSensor usDownLeft(50, 51);
UltrasonicSensor usDownRight(52, 53);
UltrasonicSensor usBackLeft(48, 49);
IMUSensor imu;
IRSensor ir1(30);
IRSensor ir2(31);
IRSensor ir3(32);
IRSensor ir4(33);
ButtonSensor button(12);
int linefollowerPins[] = {A0, A1, A2, A3, A4};
LineFollowerSensor lf(linefollowerPins, 5);

/********************
 * Actuators
*********************/
LED led(10, 9, 8);

/********************
 * Update agents
*********************/
AnglingUpdateAgent angleAgent(&state, &imu);
ButtonUpdateAgent buttonAgent(&state, &button);

/********************
 * Action agents
*********************/
LEDActionAgent ledAgent(&state, &led);


void setup() {
  
    Wire.endTransmission(true); //otherwise it doesnt work

    Serial.begin(9600);

    if(DEBUG) {
        Serial.println("Setup done");
    }
    
}

void loop() {

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

    /* TODO List:
    UpdateAgents:
    EdgeDetection
    ObstacleDetection
    LineDetection
    CircleDetection
    LoopDetection
    E-stopRequest (-> ROS)
    TicTacDropRequest (->ROS)

    ActionAgents:
    TicTacDropper +  reload mode
    Driver (Navigation,     - initial search direction (i.e. which line to follow first))
    Shutting down the robot Agent?
    ROS debugging agent, send state.toString all x seconds or if something changes
    display agent

    Actuators:
    stepper motor
    motor
    display

    Sensor:
    error correction: weighted average values over several measurements, make sure no negative distances,... occur
    
    State Machine:
    update Armed/ not armed

    Constants:
    all the pins here
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