
#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <Arduino.h>

#define BAUD_RATE 9600

#define ROBOT_SPEED 12
#define RATIO_FAST_TO_SLOW_MOTOR 1.3

#define MAX_VELOCITY 1

#define US_EDGE_THRESHOLD 22
#define US_OBSTACLE_THRESHOLD 10

#define DEBUG_PRINTING_DELAY 1000
#define DEBUG 1
#define LED_ACTION_AGENT_DEBUG 1
#define NAVIGATION_ACTION_AGENT_DEBUG 1
#define TICTAC_ACTION_AGENT_DEBUG 1
#define BUTTON_UP_UPDATE_AGENT_DEBUG 1
#define CIRCLE_DETECTION_UPDATE_AGENT_DEBUG 1
#define ENTITY_DETECTION_UPDATE_AGENT_DEBUG 1
#define INCLINE_UPDATE_AGENT_DEBUG 1
#define LINE_DETECTION_UPDATE_AGENT_DEBUG 1
#define LOOP_DETECTION_UPDATE_AGENT_DEBUG 1
#define TICTAC_UPDATE_AGENT_DEBUG 1

#define DEGREES_CIRCLES 360

#define NOT_MOVING_DELAY_AFTER_START 3000

/********************
 * ROS
 *********************/
#define ROS 1
#define ROS_ESTOP_TOPIC "ESTOP"
#define ROS_DROP_TOPIC "DROPTICTAC"

/********************
 * Pins
*********************/
#define US_NW_FORWARD_TRIGGER_PIN 44
#define US_NW_FORWARD_ECHO_PIN 45

#define US_NE_FORWARD_TRIGGER_PIN 46
#define US_NE_FORWARD_ECHO_PIN 47

#define US_W_FORWARD_TRIGGER_PIN 42
#define US_W_FORWARD_ECHO_PIN 43

#define US_NNW_DOWN_TRIGGER_PIN 48
#define US_NNW_DOWN_ECHO_PIN 49

#define US_NNE_DOWN_TRIGGER_PIN 50
#define US_NNE_DOWN_ECHO_PIN 51

#define US_SW_DOWN_TRIGGER_PIN 40
#define US_SW_DOWN_ECHO_PIN 41

#define IR_NW_PIN 30
#define IR_NE_PIN 31
#define IR_SW_PIN 32
#define IR_SE_PIN 33

#define LINE_FOLLOWER_PIN0 A4
#define LINE_FOLLOWER_PIN1 A3
#define LINE_FOLLOWER_PIN2 A2
#define LINE_FOLLOWER_PIN3 A1
#define LINE_FOLLOWER_PIN4 A0

#define START_BUTTON_PIN 23

#define LED_RED_PIN 2
#define LED_GREEN_PIN 3
#define LED_BLUE_PIN 4

#define STEPPER_MOTOR_PIN1 22
#define STEPPER_MOTOR_PIN2 24
#define STEPPER_MOTOR_PIN3 26
#define STEPPER_MOTOR_PIN4 28

#define MOTOR_PIN1 8
#define MOTOR_PIN2 9
#define MOTOR_ENA1 6

#define MOTOR_PIN3 10
#define MOTOR_PIN4 11
#define MOTOR_ENA2 7

#endif