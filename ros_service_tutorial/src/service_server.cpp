#include "ros/ros.h"	
// ServiceMessage Service File Header (Automatically created after build)
#include "ros_service_tutorial/ServiceMessage.h"

// The below process is performed when there is a service request
bool calculationCallback(ros_service_tutorial::ServiceMessage::Request &request,
                         ros_service_tutorial::ServiceMessage::Response &response)
{
	// The service name is 'ros_service' and 
    // it will call 'calculationCallback' function upon the service request
	response.result = request.num_1 + request.num_2;

	// Displays 'num_1' and 'num_2' values used in the service request and
	// the 'result' value corresponding to the service response
	ROS_INFO("Request: x=%ld, y=%ld", (long int)request.num_1, (long int)request.num_2);
	ROS_INFO("Sending back response: %ld", (long int)response.result);

	return true;
}

// Node main function
int main(int argc, char **argv) 			
{
    // Initializes node name
	ros::init(argc, argv, "service_server"); 	 

    // Node handle declaration for communication with ROS system
	ros::NodeHandle nh; 	 

	// Declare service server's name as 'ros_service_server'
	// Use the 'ServiceMessage' service file in the 'ros_service_tutorial' package
	// The service name is 'ros_service' and it will call 'calculationCallback' function
	// upon the service request.

	ros::ServiceServer ros_service_server = nh.advertiseService("ros_service", calculationCallback);

    // Print if the service server is ready
	ROS_INFO("Ready ROS Service Server!");

	// Wait for the service request
	ros::spin();
					
	return 0;

}



