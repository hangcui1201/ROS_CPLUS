#include "ros/ros.h"	
#include "ros_parameter_tutorial/ServiceParamMessage.h"

#define PLUS               1 // Addition
#define MINUS              2 // Subtraction
#define MULTIPLICATION     3 // Multiplication
#define DIVISION           4 // Division

int g_operator = PLUS;

// The below process is performed when there is a service request
bool calculationCallback(ros_parameter_tutorial::ServiceParamMessage::Request &request,
                         ros_parameter_tutorial::ServiceParamMessage::Response &response)
{
	// The operator will be selected according to the parameter value and calculate 'num_1' and 'num_2',
	// which were received upon the service request.
	// The result is stored as the response value.
	switch(g_operator)
	{
		case PLUS:
			response.result = request.num_1 + request.num_2; 
			break;

		case MINUS:
			response.result = request.num_1 - request.num_2; 
			break;

		case MULTIPLICATION:
			response.result = request.num_1 * request.num_2; 
			break;

		case DIVISION:
			if(request.num_2 == 0)
			{
				response.result = 0; 
				break;
			}
			else
			{
				response.result = request.num_1 / request.num_2; 
				break;
			}

		default:
			response.result = request.num_1 + request.num_2; 
			break;
	}

	// Displays 'num_1' and 'num_2' values used in the service request and
	// the 'result' value corresponding to the service response
	ROS_INFO("Request: x=%ld, y=%ld", (long int)request.num_1, (long int)request.num_2);
	ROS_INFO("Sending back response: %ld", (long int)response.result);

	return true;

}


int main(int argc, char **argv)
{
	// Initializes node name
	ros::init(argc, argv, "service_param_server"); 		 
	
	// Node handle declaration for communication with ROS system
	ros::NodeHandle nh; 				 

	// Reset Parameter Settings
    // Set parameter "calculation_method = PLUS"
	nh.setParam("calculation_method", PLUS);		 


	// Declare service server's name as 'ros_service_param_server'
	// Use the 'ServiceParamMessage' service file in the 'ros_parameter_tutorial' package
	// The service name is 'ros_service_param' and it will call 'calculationCallback' function
	// upon the service request.
	ros::ServiceServer ros_service_param_server = nh.advertiseService("ros_service_param", calculationCallback);

    // Print if the service server is ready
	ROS_INFO("Ready ROS Service Param Server!");

	// 10hz
	ros::Rate r(10);		
	
	while (ros::ok())
	{
		// Select the operator according to the value received from the parameter
		// Get the parameter value from calculation_method
        // Set the value of g_operator
		// g_operator checks the parameter value in every 0.1 seconds to determine
        // which operation to use on the values received through the service request
		nh.getParam("calculation_method", g_operator);

		// Callback function process routine
		ros::spinOnce(); 	

		// Sleep for routine iteration
		r.sleep(); 		
	}

	return 0;

}




























