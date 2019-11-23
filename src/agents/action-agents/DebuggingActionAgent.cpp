#include "DebuggingActionAgent.hpp"
#include "ActionAgent.hpp"

#define MIN_TIME_BETWEEN_MESSAGES 1000

/*DebuggingActionAgent::DebuggingActionAgent(State* state, ros::NodeHandle* nh): ActionAgent(state), nh(nh) {
    this->message = "This is a debugging message";
    this->publisher = ros::Publisher("receiver_data", &str_msg);
    this->nh.advertise(publisher);

    this->lastMessage = 0;
}

void DebuggingActionAgent::enact() {
    
    if(now - this->lastMessage > MIN_TIME_BETWEEN_MESSAGES) {
    // Fill the ROS message with data
    this->debuggingMessage.data = message;

    // Publish our ROS message
    publisher.publish( &debuggingMessage );
    }
}*/