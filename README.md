## ROS Basic Tutorials (C++)

### Lesson 1: ROS Topic Tutorial

#### Terminal 1
$ source devel/setup.bash  
$ roscore  

#### Terminal 2
$ source devel/setup.bash  
$ rosrun ros_topic_tutorial topic_publisher  

#### Terminal 3
$ source devel/setup.bash  
$ rosrun ros_topic_tutorial topic_subscriber  

#### Alternative
$ roslaunch ros_topic_tutorial ros_topic.launch --screen  
$ rosnode list  
$ rosrun rqt_graph rqt_graph  


### Lesson 2: ROS Service Tutorial

#### Terminal 1
$ source devel/setup.bash  
$ roscore  

#### Terminal 2
$ source devel/setup.bash  
$ rosrun ros_service_tutorial service_server  

#### Terminal 3
$ source devel/setup.bash  
$ rosrun ros_service_tutorial service_client 11 12  

#### Alternative 1
$ source devel/setup.bash  
$ roscore  
$ rosrun ros_service_tutorial service_server  
$ rosservice call /ros_service 11 12  
 
#### Alternative 2
$ source devel/setup.bash  
$ roscore  
$ rosrun ros_service_tutorial service_server  
$ rqt  
$ [Plugins] -> [Services] -> [Service Caller] -> Input num_1=10, num_2=11 -> Click "Call" button

### Lesson 3: ROS Action Tutorial

#### Terminal 1
$ source devel/setup.bash  
$ roscore  

#### Terminal 2
$ source devel/setup.bash  
$ rosrun ros_action_tutorial action_server  

#### Terminal 3
$ source devel/setup.bash  
$ rosrun ros_action_tutorial action_client   


### Lesson 4: ROS Parameters Tutorial

#### Terminal 1
$ source devel/setup.bash  
$ roscore  

#### Terminal 2
$ source devel/setup.bash  
$ rosrun ros_parameter_tutorial service_param_server  

#### Terminal 3
$ source devel/setup.bash  
$ rosrun ros_parameter_tutorial service_param_client 10 5  

#### Terminal 4

$ rosservice list  
$ rosservice call /ros_service_param 10 5  

$ rosparam set /calculation_method 2  
$ rosservice call /ros_service_param 10 5  

$ rosparam set /calculation_method 3  
$ rosservice call /ros_service_param 10 5  

$ rosparam set /calculation_method 4  
$ rosservice call /ros_service_param 10 5  

### Lesson 5: ROS Image Tutorial

#### Terminal 1
$ source devel/setup.bash  
$ roslaunch usb_cam usb_cam.launch

#### Terminal 2
$ source devel/setup.bash  
$ rosrun ros_image_tutorial image_pub_sub





























