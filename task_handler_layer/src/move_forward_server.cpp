#include <move_forward_server.h>

moveForward::moveForward(int pwm_): upwardPIDClient_("upwardPID"), anglePIDClient_("turnPID"),
                                    sidewardPIDClient_("sidewardPID") 
{
    angle_sub_ = nh.subscribe("/mavros/imu/yaw", 1, &moveForward::imuAngleCB, this);
    depth_sub_ = nh.subscribe("/varun/sensors/depth", 1, &moveForward::depthCB, this);
    nh.setParam("/pwm_forward_right", pwm_);
    nh.setParam("/pwm_forward_left", pwm_);

    depthGoalReceived = false;
    angleGoalReceived = false;
    spin_thread_ = new boost::thread(boost::bind(&moveForward::spinThread_, this));
}

moveForward::~moveForward() {
}

void moveForward::setActive(bool status) {
    if (status == true) {
        spin_thread = new boost::thread(boost::bind(&moveForward::spinThread, this));
    }

    if (status == false) {
        if (!depthGoalReceived) {
            upwardPIDClient_.cancelGoal();
        }
        if (!angleGoalReceived) {
            anglePIDClient_.cancelGoal();
        }
        sidewardPIDClient_.cancelGoal();
        spin_thread->join();
    }
}

void moveForward::spinThread() {
    ROS_INFO("Waiting for sidewardPID server to start.");
    sidewardPIDClient_.waitForServer();

    ROS_INFO("sidewardPID server started, sending goal.");
    sideward_PID_goal.target_distance = 0;
    sidewardPIDClient_.sendGoal(sideward_PID_goal);

    //////////////////////////////////////////////////////

    ROS_INFO("Waiting for upwardPID server to start.");
    upwardPIDClient_.waitForServer();

    while (depthGoalReceived) {}

    ROS_INFO("upwardPID server started, sending goal.");
    upward_PID_goal.target_depth = depth;
    upwardPIDClient_.sendGoal(upward_PID_goal);

    ///////////////////////////////////////////////////////

    ROS_INFO("Waiting for anglePID server to start.");
    anglePIDClient_.waitForServer();
    
    while (angleGoalReceived) {}

    ROS_INFO("anglePID server started, sending goal.");
    angle_PID_goal.target_angle = angle;
    anglePIDClient_.sendGoal(angle_PID_goal);
}

void moveForward::spinThread_() {
    ros::spin();
}

void moveForward::setReferenceAngle(double angle_) {
    angle = angle_;

    ROS_INFO("anglePID reset, sending goal.");
    angle_PID_goal.target_angle = angle;
    anglePIDClient_.sendGoal(angle_PID_goal);
}

void moveForward::setReferenceDepth(double depth_) {
    depth = depth_;

    ROS_INFO("upwardPID reset, sending goal.");
    upward_PID_goal.target_depth = depth;
    upwardPIDClient_.sendGoal(upward_PID_goal);
}

void moveForward::imuAngleCB(const std_msgs::Float64Ptr &_msg) {
    angle = _msg->data;
    angleGoalReceived = true;
}

void moveForward::depthCB(const std_msgs::Float64Ptr &_msg) {
    depth = _msg->data;
    depthGoalReceived = true;
}