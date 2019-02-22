# Graphical User Interface for ROS
## Humanoid Robotics, IITK
### Usage Instructions

__Clone__ the repostitory

To begin, we will launch ROS. To do so, run the following in a terminal:
   ` roscore`
   
Next, start publishing a message from the server to test our JavaScript subscriber:
`    rostopic pub /listener std_msgs/String "Hello, World"`

Then start subscribing to a topic to test publishing from the browser:
`    rostopic echo /cmd_vel`

Now run a service server that will return service calls from the browser:
   ` rosrun rospy_tutorials add_two_ints_server`

Once everything is running, we can launch the rosbridge v2.0 server with the following:
    `roslaunch rosbridge_server rosbridge_websocket.launch`

