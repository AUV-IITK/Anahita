#include <single_buoy.h>

singleBuoy::singleBuoy(): forwardPIDClient("forwardPID"), sidewardPIDClient("sidewardPID"), anglePIDClient("turnPID") {
    forward_sub_ = nh_.subscribe("/buoy_task/buoy_coordinates", 1, &singleBuoy::forwardCB, this);
    sideward_sub_ = nh_.subscribe("/buoy_task/buoy_coordinates", 1, &singleBuoy::sidewardCB, this);
    angle_sub_ = nh_.subscribe("/varun/sensors/imu/yaw", 1, &singleBuoy::angleCB, this);    
}
singleBuoy::~singleBuoy() {}

void singleBuoy::setActive(bool status) {

    motion_layer::sidewardPIDGoal sideward_PID_goal;

    ROS_INFO("Waiting for sidewardPID server to start.");
    sidewardPIDClient.waitForServer();

    ROS_INFO("sidewardPID server started, sending goal.");
    sideward_PID_goal.target_distance = 0;
    sidewardPIDClient.sendGoal(sideward_PID_goal);

    ///////////////////////////////////////////////////

    motion_layer::anglePIDGoal angle_PID_goal;

    ROS_INFO("Waiting for anglePID server to start.");
    anglePIDClient.waitForServer();

    ROS_INFO("anglePID server started, sending goal.");
    angle_PID_goal.target_angle = 0;
    anglePIDClient.sendGoal(angle_PID_goal);

    /////////////////////////////////////////////////////

    nh_.setParam("/pwm_forward_right", 100);
    nh_.setParam("/pwm_forward_left", 100);

    while(forward_distance_ >= 60) {
        continue;
    }
    sidewardPIDClient.cancelGoal();
    ros::Duration(6).sleep();

    nh_.setParam("/pwm_forward_right", -100);
    nh_.setParam("/pwm_forward_left", -100);

    ros::Duration(10).sleep();

    //////////////////////////////////////////////////////

    ROS_INFO("Waiting for action server to start.");
    forwardPIDClient.waitForServer();

    ROS_INFO("Action server started, sending goal.");
    
    motion_layer::forwardPIDGoal forwardPIDgoal;
    forwardPIDgoal.target_distance = 100;
    forwardPIDClient.sendGoal(forwardPIDgoal);
}

void singleBuoy::forwardCB(const geometry_msgs::PointStamped::ConstPtr &_msg) {
    forward_distance_ = _msg->point.x;
}

void singleBuoy::sidewardCB(const geometry_msgs::PointStamped::ConstPtr &_msg) {
    sideward_distance_ = _msg->point.y;
}

void singleBuoy::angleCB(const std_msgs::Float64Ptr &_msg) {
    angle_ = _msg->data;
}
