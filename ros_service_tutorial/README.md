#### Open the first terminal  

$ cd ~/catkin_basic_py  
$ catkin_make  
$ source devel/setup.bash  
$ roscore  

#### Open a second terminal  

$ cd ~/catkin_basic_py  
$ source devel/setup.bash  
$ rosrun ros_service_tutorial service_server

#### Open a third terminal  

$ cd ~/catkin_basic_py  
$ source devel/setup.bash  
$ rosrun ros_service_tutorial service_client 11 12  

or  

$ roscore  
$ rosrun ros_service_tutorial service_server  
$ rosservice call /ros_service 11 12  

or  

$ roscore  
$ rosrun ros_service_tutorial service_server  
$ rqt  
$ [Plugins] -> [Services] -> [Service Caller] -> Input num_1=10, num_2=11 -> Click "Call" button
