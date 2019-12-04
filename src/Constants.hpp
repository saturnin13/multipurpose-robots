
#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <Arduino.h>

#define BAUD_RATE 9600

#define ROBOT_SPEED 15
#define RATIO_FAST_TO_SLOW_MOTOR 2.0
#define RATIO_INCLINE 1.5
#define RATIO_DECLINE 1.5

#define MAX_VELOCITY 1

#define US_EDGE_THRESHOLD 22
#define US_OBSTACLE_THRESHOLD 30

#define DEBUG_PRINTING_DELAY 1000
#define DEBUG 1
#define LED_ACTION_AGENT_DEBUG 0
#define NAVIGATION_ACTION_AGENT_DEBUG 1
#define TICTAC_ACTION_AGENT_DEBUG 0
#define BUTTON_UP_UPDATE_AGENT_DEBUG 1
#define CIRCLE_DETECTION_UPDATE_AGENT_DEBUG 0
#define ENTITY_DETECTION_UPDATE_AGENT_DEBUG 0
#define INCLINE_UPDATE_AGENT_DEBUG 0
#define LINE_DETECTION_UPDATE_AGENT_DEBUG 0
#define LOOP_DETECTION_UPDATE_AGENT_DEBUG 0
#define TICTAC_UPDATE_AGENT_DEBUG 0

#define DEGREES_CIRCLE 360

#define ARMING_DELAY 3000

#define TICTAC_TIME_RUNNING_AFTER_INCLINE 1000
#define TOTAL_DROPPING_TIME 8000

#define LINE_FOLLOWING_COMPLETED_DELAY 3000

/********************
 * ROS
 *********************/
#define ROS 0
#define ROS_ESTOP_TOPIC "ESTOP"
#define ROS_DROP_TOPIC "DROPTICTAC"

/********************
 * Pins
*********************/
#define US_NW_FORWARD_TRIGGER_PIN 26
#define US_NW_FORWARD_ECHO_PIN 27

#define US_NE_FORWARD_TRIGGER_PIN 28
#define US_NE_FORWARD_ECHO_PIN 29

#define US_W_FORWARD_TRIGGER_PIN 30
#define US_W_FORWARD_ECHO_PIN 31

#define US_NW_DOWN_TRIGGER_PIN 24
#define US_NW_DOWN_ECHO_PIN 25

#define US_NE_DOWN_TRIGGER_PIN 36
#define US_NE_DOWN_ECHO_PIN 37

#define US_SW_DOWN_TRIGGER_PIN 22
#define US_SW_DOWN_ECHO_PIN 23

#define IR_NW_PIN 33
#define IR_NE_PIN 52
#define IR_SW_PIN 32
#define IR_SE_PIN 53

#define LINE_FOLLOWER_PIN0 A11
#define LINE_FOLLOWER_PIN1 A12
#define LINE_FOLLOWER_PIN2 A13
#define LINE_FOLLOWER_PIN3 A14
#define LINE_FOLLOWER_PIN4 A15

#define START_BUTTON_PIN 34

#define LED_RED_PIN 2
#define LED_GREEN_PIN 3
#define LED_BLUE_PIN 4

#define STEPPER_MOTOR_PIN1 48
#define STEPPER_MOTOR_PIN2 46
#define STEPPER_MOTOR_PIN3 44
#define STEPPER_MOTOR_PIN4 42

#define MOTOR_PIN1 47
#define MOTOR_PIN2 49
#define MOTOR_ENA1 12

#define MOTOR_PIN3 45
#define MOTOR_PIN4 43
#define MOTOR_ENA2 13

#endif