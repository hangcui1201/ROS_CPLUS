#### Open the first terminal  

$ cd ~/catkin_basic_py  
$ catkin_make  
$ source devel/setup.bash  
$ roscore  

#### Open a second terminal  

$ cd ~/catkin_basic_py  
$ source devel/setup.bash
$ rosrun ros_topic_tutorial topic_publisher  


#### Open a third terminal  

$ cd ~/catkin_basic_py  
$ source devel/setup.bash
$ rosrun ros_topic_tutorial topic_subscriber  

or

$ roslaunch ros_topic_tutorial ros_topic.launch --screen  
$ rosnode list  
$ rosrun rqt_graph rqt_graph  
