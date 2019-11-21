#include <Arduino.h>
#include<Wire.h>

#include "actuators/LED.hpp"

#include "sensors/ButtonSensor.hpp"
#include "sensors/IMUSensor.hpp"
#include "sensors/IRSensor.hpp"
#include "sensors/LineFollowerSensor.hpp"
#include "sensors/UltrasonicSensor.hpp"

#include "agents/ButtonUpdateAgent.hpp"
#include "agents/AnglingUpdateAgent.hpp"
#include "agents/LEDActionAgent.hpp"

#include "state_machine/StateMachine.hpp"

#define DEBUG 1
#define STARTDELAYTIME 3000


// Example of how to create ultrasonic sensors.
UltrasonicSensor usFrontLeft(46, 47);
UltrasonicSensor usFrontRight(44, 45);
UltrasonicSensor usFrontMiddle(42, 43);
UltrasonicSensor usDownLeft(50, 51);
UltrasonicSensor usDownRight(52, 53);
UltrasonicSensor usBackLeft(48, 49);

// Example of how to create IMU
IMUSensor imu;

// Example of how to create IR sensor
IRSensor ir1(30);
IRSensor ir2(31);
IRSensor ir3(32);
IRSensor ir4(33);

// Example of how to a button
ButtonSensor button(12);

// Example of LED
LED led(10, 9, 8);

// Example of how to create line follower sensor
int linefollowerPins[] = {A0, A1, A2, A3, A4};
LineFollowerSensor lf(linefollowerPins, 5);

//State
State state;

//Example of new update Agent
AnglingUpdateAgent angleAgent(&state, &imu);
ButtonUpdateAgent buttonAgent(&state, &button);

//Example of new action Agent
LEDActionAgent ledAgent(&state, &led);

//time stracking
long ledTime;


void setup() {
  
    Wire.endTransmission(true); //otherwise it doesnt work

    Serial.begin(9600);

    if(DEBUG) {
        Serial.println("Setup done");
    }
    
}

void loop() {
    button.update();
    //DISCUSS: update imu in angle Agent?
    //PRO: is then updated automataically,
    //CON: some sensors are used in more than one agent -> who should update then?
    imu.update();

    /*TODO List:
    UpdateAgents:
    EdgeDetection
    ObstacleDetection
    LineDetection
    CircleDetection
    LoopDetection
    E-stopRequest (-> ROS)
    TicTacDropRequest (->ROS)

    ActionAgents:
    TicTacDropper + Stepper Motor
    Driver + Motor

    Question:
    Where is Navigation going?

    Add to states:
    - stop (not e stop, but for dropping)
    - drop request
    - initial search direction (i.e. which line to follow first)
    - navigation mode (left all the time, search for circle, line following)
    - max velocity (?)
    */
    

    //usFrontLeft.update();
    //usFrontMiddle.update();
    //usFrontRight.update();
    //usDownLeft.update();
    //usDownRight.update();
    //usBackLeft.update();
    //ir1.update();
    //ir2.update();
    //ir3.update();
    //ir4.update();
    //lf.update();

    angleAgent.update();
    buttonAgent.update();
    ledAgent.enact();    


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

    // Update state machine machine

    // Carry out actions
}