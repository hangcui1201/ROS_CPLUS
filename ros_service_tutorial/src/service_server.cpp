#include "ros/ros.h"	
// MsgService Service File Header (Automatically created after build)
#include "ros_service_tutorial/MsgService.h"

// The below process is performed when there is a service request
bool calculationCallback(ros_service_tutorial::MsgService::Request &request,
                         ros_service_tutorial::MsgService::Response &response)
{
	// The service name is 'ros_tutorial_srv' and 
    // it will call 'calculation' function upon the service request.
	response.result = request.a + request.b;

	// Displays 'a' and 'b' values used in the service request and
	// the 'result' value corresponding to the service response
	ROS_INFO("request: x=%ld, y=%ld", (long int)request.a, (long int)request.b);
	ROS_INFO("sending back response: %ld", (long int)response.result);

	return true;
}

// Node Main Function
int main(int argc, char **argv) 			
{
    // Initializes Node Name
	ros::init(argc, argv, "service_server"); 	 

    // Node handle declaration for communication with ROS system
	ros::NodeHandle nh; 	 

	// Declare service server 'ros_service_server'
	// using the 'MsgService' service file in the 'ros_service_tutorial' package
	// The service name is 'ros_srv_server' and it will call 'calculation' function
	// upon the service request.

	ros::ServiceServer ros_service_server = nh.advertiseService("ros_service", calculationCallback);

	ROS_INFO("Ready ROS Service Server!");

	// Wait for the service request
	ros::spin();
					
	return 0;

}



