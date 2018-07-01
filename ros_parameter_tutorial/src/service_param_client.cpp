#include "ros/ros.h"	
// ServiceMessage Service File Header (Automatically created after build)
#include "ros_parameter_tutorial/ServiceParamMessage.h"
// Library for using the "atoll" function
#include <cstdlib>
					 
// Node Main Function
int main(int argc, char **argv)			 
{
	// Initializes Node Name
	ros::init(argc, argv, "service_param_client");	
	
	// input value error handling
	if (argc != 3)				
	{
		ROS_INFO("Command: rosrun ros_parameter_tutorial service_param_client arg0 arg1");
		ROS_INFO("arg0: double number, arg1: double number");
		return 1; 
	}

	// Node handle declaration for communication with ROS system
	ros::NodeHandle nh; 	

	// Declares service client 'ros_service_param_client'
	// Use the 'ServiceParamMessage' service file in the 'ros_parameter_tutorial' package
	// The service name is 'ros_service_param'
	ros::ServiceClient ros_service_param_client = nh.serviceClient<ros_parameter_tutorial::ServiceParamMessage>("ros_service_param");

	// Declares the 'srv' service that uses the 'ServiceParamMessage' service file
	ros_parameter_tutorial::ServiceParamMessage srv;

	// Parameters entered when the node is executed as a service request value are stored at 'num_1' and 'num_2'
	srv.request.num_1 = atoll(argv[1]);
	srv.request.num_2 = atoll(argv[2]);

	// Request the service. If the request is accepted, display the response value
	if (ros_service_param_client.call(srv))
	{
		ROS_INFO("Send srv, srv.Request.num_1 and num_2: %ld, %ld", (long int)srv.request.num_1, (long int)srv.request.num_2);
		ROS_INFO("Receive srv, srv.Response.result: %ld", (long int)srv.response.result);
	}
	else
	{
		ROS_ERROR("Failed to call service ros_service_param!");
		return 1;
	}

	return 0;
}





