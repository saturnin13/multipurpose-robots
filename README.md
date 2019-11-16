# multipurpose-robots
A mutipurpose robot built from scratch that can drive itself successfully in a challenging environment.

# Ros, Pi and Ardunio
## Connecting the Computer via Wifi/ssh to the RaspberryPi, using Ros to talk with the arudino
Our goal is to push a button on a computer to trigger some actions on the Arduino.
Therefore, the computer is connected to the Pi via ssh. The Pi runs ros. The Arduino has a special script that subscribes to the Pi's ros. Now if we send the correct message to the arduino, an
action can be triggered.

## How to do it
1. prepare a arduino file to subscribe (see example rosBlink_test on GitHub)
2. Put the file in the folder arduino examples (or don't do it, but i did it this way)
3. change the file flash_arduino.sh from the experimental robotics GitHub to flash the new file
(so open it and change the file name accordingly)
4. Now we have to copy the new flash_arduino.sh and the arduino file to the pi to flash it to the arduino thereafter

## Copy the files
0. go to
``bash
cd ~/MECSEE129/arduino_examples/
``

1. copy the file flash_arduino.sh to the pi@pibennysat an from the directory
``bash
scp flash_arduino.sh pi@pibennysat:"/Users/bennyroder/Desktop/Experimental\\ Robotics/Pi\\ Lab/MECSEE129-master/arduino_examples"
``

2. copy the file (ATTENTION: i renamed the file from rosBlink_test.ino to subscriber_example.ino)
``bash
scp subscriber_example.ino 192.168.1.124:/Users/bennyroder/Desktop/Experimental\ Robotics/Pi\ Lab/MECSEE129-master/arduino_examples/
``

3. Use
``bash
ls``
to see if the files show up

## flash the Arduino
``bash
./flash_arduino.sh
``
Its just that


## Terminal 1
1. First do
``bash
cd ~/catkin_ws/
``

2. Then do
``bash
source ~/catkin_ws/devel/setup.bash
``

3. and
``bash
rosrun rosserial_arduino serial_node.py_port:=/dev/ttyArduino
``

## Terminal 3
1. ssh into the pi
``bash
ssh pi@192.168.1.124
``

2. start roscore
``bash
roscore
``

3. Source the catkin thing
``bash
source ~/catkin_ws/devel/setup.bash
``

4. Show all topics (should cointain /toggle_led for the given example)
``bash
rostopic list
``

5. This line will trigger the action (light LED in the example) on the arduino, as it is the specified message
``bash
rostopic pub toggle_led std_msgs/Empty --once
``

_Now we expect the arduino to light one LED up (onboard LED)_
If we send the last message (5.) again, the LED should turn off.
For the future, we can change the line
``cpp
digitalWrite(13, HIGH-digitalRead(13)
``
to whatever we want to trigger from the PC (e.g. emergency stop or Tic Tac dropping)
