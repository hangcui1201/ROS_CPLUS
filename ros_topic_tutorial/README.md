# To Run this package and nodes

Put 'ros_topic_tutorial' folder under the 'src' folder of catkin workspace

>> cd 'catkin_workspace'
>> catkin_make
>> source devel/setup.bash
>> roscore
>> rosrun ros_topic_tutorial topic_publisher
>> rosrun ros_topic_tutorial topic_subscriber

or

>> roslaunch ros_topic_tutorial ros_topic.launch --screen
>> rosnode list
>> rosrun rqt_graph rqt_graph
