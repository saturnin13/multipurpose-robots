
#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <Arduino.h>

#define ROBOT_SPEED 12

#define MAX_VELOCITY 1
#define START_DELAY_TIME 3000

#define US_EDGE_THRESHOLD 2
#define US_OBSTACLE_THRESHOLD 5

#define DEBUG 0

#define DEGREES_CIRCLES 360

#define IR_THRESHOLD 500

/********************
 * ROS
 *********************/
#define ROS 1
#define ROS_ESTOP_TOPIC "ESTOP"
#define ROS_DROP_TOPIC "DROPTICTAC"

/********************
 * Pins
*********************/
#define US_NW_FORWARD_TRIGGER_PIN 46
#define US_NW_FORWARD_ECHO_PIN 47

#define US_NE_FORWARD_TRIGGER_PIN 44
#define US_NE_FORWARD_ECHO_PIN 45

#define US_N_FORWARD_TRIGGER_PIN 42
#define US_N_FORWARD_ECHO_PIN 43

#define US_NNW_DOWN_TRIGGER_PIN 48
#define US_NNW_DOWN_ECHO_PIN 49

#define US_NNE_DOWN_TRIGGER_PIN 50
#define US_NNE_DOWN_ECHO_PIN 51

#define US_SW_DOWN_TRIGGER_PIN 52
#define US_SW_DOWN_ECHO_PIN 53

#define IR_NW_PIN 30
#define IR_NE_PIN 31
#define IR_SW_PIN 32
#define IR_SE_PIN 33

#define LINE_FOLLOWER_PIN0 A0
#define LINE_FOLLOWER_PIN1 A1
#define LINE_FOLLOWER_PIN2 A2
#define LINE_FOLLOWER_PIN3 A3
#define LINE_FOLLOWER_PIN4 A4

#define START_BUTTON_PIN 23

#define LED_RED_PIN 22
#define LED_GREEN_PIN 2
#define LED_BLUE_PIN 3

#define STEPPER_MOTOR_PIN1 10
#define STEPPER_MOTOR_PIN2 11
#define STEPPER_MOTOR_PIN3 12
#define STEPPER_MOTOR_PIN4 13

#define MOTOR_PIN1 4
#define MOTOR_PIN2 5
#define MOTOR_ENA1 8

#define MOTOR_PIN3 6
#define MOTOR_PIN4 7
#define MOTOR_ENA2 9

#endif