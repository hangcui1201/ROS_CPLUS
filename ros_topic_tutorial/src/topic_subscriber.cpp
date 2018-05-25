#include "ros/ros.h"

// MsgTutorial Message File Header
// The header file is automatically created when building the package.
#include "ros_topic_tutorial/MsgTopic.h"

// Message callback function. This is a function is called when a topic
// message named 'ros_msg_topic' is received.
void msgCallback(const ros_topic_tutorial::MsgTopic::ConstPtr& msg){

    // Shows the 'stamp.sec' message
    ROS_INFO("recieve msg = %d", msg->stamp.sec);	
    
    // Shows the 'stamp.nsec' message
    ROS_INFO("recieve msg = %d", msg->stamp.nsec);

    // Shows the 'data' message	 
    ROS_INFO("recieve msg = %d", msg->data);		 
}

int main(int argc, char **argv){

    // package name: ros_topic_tutorial
    // node name: topic_subscriber
    // subscriber: ros_sub_topic
    // topic name: ros_msg_topic
    // message file: MsgTopic.msg

    // Initializes node name as "topic_subscriber"
    ros::init(argc, argv, "topic_subscriber"); 	
    
    // Node handle declaration for communication with ROS system
    ros::NodeHandle nh; 	

    // Declares subscriber. Create subscriber 'ros_sub_topic' using the MsgTopic.msg
    ros::Subscriber ros_sub_topic = nh.subscribe("ros_msg_topic", 100, msgCallback);

    // A function for calling a callback function, waiting for a message to be
    // received, and executing a callback function when it is received
    ros::spin();

    return 0;

}











