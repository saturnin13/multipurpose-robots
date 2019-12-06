# Ros, Pi and Ardunio
## Connecting the Computer via Wifi/ssh to the RaspberryPi, using Ros to talk with the arudino
Our goal is to push a button on a computer to trigger some actions on the Arduino.
Therefore, the computer is connected to the Pi via ssh. The Pi runs ros. The Arduino has a special script that subscribes to the Pi's ros. Now if we send the correct message to the arduino, an
action can be triggered.

## How to do it
1. prepare a arduino file to subscribe (see example ROS_test on GitHub)
1. Put the file in the folder arduino examples (or don't do it, but i did it this way)
1. change the file flash_arduino.sh from the experimental robotics GitHub to flash the new file
(so open it and change the file name accordingly, but for a file named ROS_test it just works)
1. Now we have to copy the new flash_arduino.sh and the arduino file to the pi to flash it to the arduino thereafter
1. make sure the arduino you use is the one with the sticker on the back, if not you have to change the serial number (that is a lot to do, better start looking for the correct arduino)

## Copy the files
1. go to (on your computer, the file where flash_arduino.sh and the arduino file are), so on my computer:
```bash
cd Desktop/Experimental\ Robotics/Pi\ Lab/MECSEE129-master/arduino_examples/
```

2. copy the file flash_arduino.sh to the pi@192.168.1.124 (has to be the correct IP address) an from the directory (you should know the password at this point)
```bash
scp flash_arduino.sh pi@192.168.1.124:~/MECSEE129/arduino_examples
```

1. copy the file (ATTENTION if renamed the file from ROS_test.ino to sth else)
```bash
scp ROS_test.ino pi@192.168.1.124:~/MECSEE129/arduino_examples
```

1. ssh into the pi in another terminal and move to
```bash
cd ~/MECSEE129/arduino_examples
```

1. Now use
```bash
ls
```
to see if the files show up

and flash the Arduino
```bash
./flash_arduino.sh
```
Its just that (if it has error messages, better fix your code first)

## Terminal 1
1. ssh into the pi
```bash
ssh pi@192.168.1.124
```

2. run
```bash
roscore
```

## Terminal 2
1. ssh into the pi
```bash
ssh pi@192.168.1.124
```

1. First do
```bash
cd ~/catkin_ws/
```

2. Then do
```bash
source ~/catkin_ws/devel/setup.bash
```

3. and
```bash
rosrun rosserial_arduino serial_node.py _port:=/dev/ttyArduino _baud:=57600
57600
```
If this works, we are on track, if not, probably fix the arduino code first.

## Terminal 3
1. ssh into the pi
```bash
ssh pi@192.168.1.124
```

2. Source the catkin thing
```bash
source ~/catkin_ws/devel/setup.bash
```

3. Show all topics (should contain our topics, e.g. first_action (= e-stop), second_action (= drop tic tac) and receiver_data (= get error messages or data) for the given example)
```bash
rostopic list
```

4. This line will trigger the action (light LED in the example) on the arduino, as it is the specified message
```bash
rostopic pub ESTOP std_msgs/Bool false --once
```

OR

```bash
 rostopic pub DROPTICTAC std_msgs/Empty --once
```

with first_action being the identifier for the first action.
_Now we expect the arduino to light one LED up (onboard LED)_

If we send the last message (5.) again, the LED should turn off.
For the future, we can change the line

```bash
digitalWrite(13, HIGH-digitalRead(13)
```

to whatever we want to trigger from the PC (e.g. emergency stop or Tic Tac dropping)

5. This line will give you information from the Arduino (error messages for example)
```bash
rostopic echo /receiver_data
```
_Now we expect the the arduino to print stuff to the console_
