#include "ros/ros.h"

// MsgTopic Message File Header
// The header file is automatically created when building the package
#include "ros_topic_tutorial/MsgTopic.h"

int main(int argc, char **argv){

    // package name: ros_topic_tutorial
    // node name: topic_publisher
    // publisher: ros_pub_topic
    // topic name: ros_msg_topic
    // message file: MsgTopic.msg

    // Initializes node name as "topic_publisher"
    ros::init(argc, argv, "topic_publisher");	

    // Node handle declaration for communication with ROS system
    ros::NodeHandle nh;		

    // Declare publisher, create publisher 'ros_pub_topic' using the 'MsgTopic.msg
    // The topic name is 'ros_msg_topic' and the size of the publisher queue is set to 100.
    ros::Publisher ros_pub_topic =
	nh.advertise<ros_topic_tutorial::MsgTopic>("ros_msg_topic", 100);

    // Set the loop period. '10' refers to 10 Hz
    // The main loop repeats at 0.1 second intervals
    ros::Rate loop_rate(10);

    // Declares 'MsgTopic' message object 'msg'
    ros_topic_tutorial::MsgTopic msg; 	
							
    int count = 0;					

    while(ros::ok()){

        // Save current time in the stamp of 'msg'
        msg.stamp = ros::Time::now();	 

        // Save the the 'count' value in the data of 'msg'
        msg.data = count;
 
        // Print the 'stamp.sec' message
        ROS_INFO("send msg = %d", msg.stamp.sec);	

        // Print the 'stamp.nsec' message
        ROS_INFO("send msg = %d", msg.stamp.nsec);
 
        // Print the 'data' message
        ROS_INFO("send msg = %d", msg.data);
 
        // Publishes 'msg' message
        ros_pub_topic.publish(msg);

        // Call this function often to all ROS to process incoming message
        ros::spinOnce();		
 
        // Goes to sleep according to the loop rate, 0.1s
        loop_rate.sleep();

        // Increase count variable by one
        count++;
					 
    }

    return 0;

}




	










