#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "../laserscan/LaserScanner.h"

using namespace std;

/*
	360 Laser Distance Sensor HLS-LFCD-LDS is a 2D laser scanner 
	capable of sensing 360 degrees that collects a set of data 
	around the robot to use for SLAM and Navigation. 

	/scan   --  sensor_msgs/LaserScan

	std_msgs/Header header
	  uint32 seq
	  time stamp
	  string frame_id

	float32 angle_min
	float32 angle_max
	float32 angle_increment
	float32 time_increment
	float32 scan_time
	float32 range_min
	float32 range_max
	float32[] ranges
	float32[] intensities

*/

//sensor_msgs::LaserScan _scanMsg;

//ros::Subscriber scanSubscriber;

// rosrun ros_2d_laser_tutorial laserscan_sub

void laserscanCallback (sensor_msgs::LaserScan scanMessage);

int main(int argc, char **argv){

	ros::init(argc, argv, "laser_scan_node");

	ros::NodeHandle nh;

	ros::Subscriber scanSubscriber = nh.subscribe("/scan", 10, laserscanCallback);

	ros::spin();
}

void laserscanCallback (sensor_msgs::LaserScan scanMessage){

	//_scanMsg = scanMessage;
	
	cout << "Minimum range: " << LaserScanner::getMinimumRange(scanMessage) << endl;
	cout << "Maximum range: " << LaserScanner::getMaximumRange(scanMessage) << endl;
	cout << "Average range: " << LaserScanner::getAverageRange(scanMessage, 0, 359) << endl;

    // if (LaserScanner::isObstacleTooClose(scanMessage, 0, 600, 0.69) == true){
    //     cout << "Obstacle too close!" << endl;
    // }

	cout << endl;

}
