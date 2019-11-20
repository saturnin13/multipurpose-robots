/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String str_msg;

//method for first action
void messageCa( const std_msgs::Empty& toggle_msg){
  digitalWrite(12, HIGH-digitalRead(12));   // blink the led
}

//method for second action
void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}

//a message, should be changed later...
char msg_str[16] = "Great success!A";


//first subscriber (will perform first action on demand of computer)
ros::Subscriber<std_msgs::Empty> sub("first_action", &messageCb );
//second subscriber (will perform second action on demand of computer)
ros::Subscriber<std_msgs::Empty> sub2("second_action", &messageCa );
//one publisher (will send data to computer, error messages for example)
ros::Publisher string_pub("receiver_data", &str_msg);

void setup()
{
  pinMode(13, OUTPUT);

  //ROS stuff
  nh.initNode();
  //subsribe to first and second sub
  nh.subscribe(sub);
  nh.subscribe(sub2);
  //advertise to pub
  nh.advertise(string_pub);

}

void loop()
{
  // Fill the ROS message with data
  str_msg.data = msg_str;

  // Publish our ROS message
  string_pub.publish( &str_msg );
  
  nh.spinOnce();
  delay(250);
}
