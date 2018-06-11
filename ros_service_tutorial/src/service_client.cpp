#include "ros/ros.h"	
// ServiceMessage Service File Header (Automatically created after build)
#include "ros_service_tutorial/ServiceMessage.h"
// Library for using the "atoll" function
#include <cstdlib>
					 
// Node Main Function
int main(int argc, char **argv)			 
{
	// Initializes Node Name
	ros::init(argc, argv, "service_client");	
	
	// input value error handling
	if (argc != 3)				
	{
		ROS_INFO("Command: rosrun ros_service_tutorial service_client arg0 arg1");
		ROS_INFO("arg0: double number, arg1: double number");
		return 1; 
	}

	// Node handle declaration for communication with ROS system
	ros::NodeHandle nh; 	

	// Declares service client 'ros_service_client'
	// Use the 'ServiceMessage' service file in the 'ros_service_tutorial' package
	// The service name is 'ros_service'
	ros::ServiceClient ros_service_client = nh.serviceClient<ros_service_tutorial::ServiceMessage>("ros_service");

	// Declares the 'srv' service that uses the 'MsgService' service file
	ros_service_tutorial::ServiceMessage srv;

	// Parameters entered when the node is executed as a service request value are stored at 'a' and 'b'
	srv.request.num_1 = atoll(argv[1]);
	srv.request.num_2 = atoll(argv[2]);

	// Request the service. If the request is accepted, display the response value
	if (ros_service_client.call(srv))
	{
		ROS_INFO("Send srv, srv.Request.num_1 and num_2: %ld, %ld", (long int)srv.request.num_1, (long int)srv.request.num_2);
		ROS_INFO("Receive srv, srv.Response.result: %ld", (long int)srv.response.result);
	}
	else
	{
		ROS_ERROR("Failed to call service ros_service!");
		return 1;
	}

	return 0;
}





