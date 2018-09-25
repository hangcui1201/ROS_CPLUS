#include <ros/ros.h>					
#include <actionlib/client/simple_action_client.h> 	 // action library header file
#include <actionlib/client/terminal_state.h> 	     // action goal status header file
#include <ros_action_tutorial/FibonacciAction.h>	 // FibonacciAction action file header
#include <string>

// Node Main Function
int main (int argc, char **argv) 			 
{
	// Node name initialization
	ros::init(argc, argv, "action_client"); 		

	// Action client declaration (Action Name: ros_action_tutorial)
	actionlib::SimpleActionClient<ros_action_tutorial::FibonacciAction> ac("ros_action_tutorial", true);

	ROS_INFO("Waiting for action server to start.");

	// Wait until action server starts
	ac.waitForServer(); 

	ROS_INFO("Action server started, sending goal.");

	// Declare action goal
	ros_action_tutorial::FibonacciGoal goal;	

	// Set action goal (process the Fibonacci sequence 20 times)
	goal.order = 20; 

	// Transmit action goal
	ac.sendGoal(goal); 

	// Set action time limit (set to 30 seconds)
	bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

	// Process when action results are received within the time limit for achieving the action goal
	if (finished_before_timeout)
	{
		// Receive action target status value and display on screen
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s", state.toString().c_str());
	}
	else
	{
		// If time out occurs
		ROS_INFO("Action did not finish before the time out.");
	}

	return 0;

}



















