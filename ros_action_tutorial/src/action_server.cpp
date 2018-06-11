#include <ros/ros.h>					
#include <actionlib/server/simple_action_server.h>	 // action library header file
#include <ros_action_tutorial/FibonacciAction.h> 	 // FibonacciAction action file header

class FibonacciAction
{
	protected:

		// Node handle declaration
		ros::NodeHandle nh_;

		// Action server declaration
		actionlib::SimpleActionServer<ros_action_tutorial::FibonacciAction> as_;

		// Use as action name
		std::string action_name_;

		// Declare the action feedback and the result to Publish
		ros_action_tutorial::FibonacciFeedback feedback_;
		ros_action_tutorial::FibonacciResult result_;

	public:
	
		// Initialize action server (Node handle, action name, action callback function)
		FibonacciAction(std::string name) : as_(nh_, name, boost::bind(&FibonacciAction::executeCB, this, _1), false),
                                            action_name_(name)
		{
			as_.start();
		}

		~FibonacciAction(void){}

		// A function that receives an action goal message and performs a specified
		// action (in this example, a Fibonacci calculation)
		void executeCB(const ros_action_tutorial::FibonacciGoalConstPtr &goal)
		{
			// Loop Rate: 1Hz
			ros::Rate r(1);

			// Used as a variable to store the success or failure of an action		 
			bool success = true;	 

			// Setting Fibonacci sequence initialization,
			// add first (0) and second message (1) of feedback.
			feedback_.sequence.clear();
			feedback_.sequence.push_back(0);
			feedback_.sequence.push_back(1);

			// Notify the user of action name, goal, initial two values of Fibonacci sequence
			ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i",
					 action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

			// Action content
			for(int i=1; i<=goal->order; i++)
			{
				// Confirm action cancellation from action client
				if (as_.isPreemptRequested() || !ros::ok())
				{
					// Notify action cancellation
					ROS_INFO("%s: Preempted", action_name_.c_str());

					// Action cancellation
					as_.setPreempted();	

					// Consider action as failure and save to variable	 
					success = false;
		 
					break;
				}

				// Store the sum of current Fibonacci number and the previous number in the feedback
				// while there is no action cancellation or the action target value is reached.
				feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);

				// Publish feedback
				as_.publishFeedback(feedback_);	

				// sleep according to the defined loop rate	 
				r.sleep();					 

			}

			// If the action target value is reached,
			// transmit current Fibonacci sequence as the result value.
			if(success)
			{
				result_.sequence = feedback_.sequence;
				ROS_INFO("%s: Succeeded", action_name_.c_str());
				as_.setSucceeded(result_);
			}

		}

};

// Node main function
int main(int argc, char** argv)			
{
	ros::init(argc, argv, "action_server");		

	// Initializes Node Name
	// Fibonacci Declaration(Action Name: ros_action_tutorial)
	FibonacciAction fibonacci("ros_action_tutorial");

	ros::spin();					
	
	return 0;

}










