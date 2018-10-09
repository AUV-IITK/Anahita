#include <move_forward_server.h>

moveForward::moveForward(int pwm_): upwardPIDClient_("upwardPID"), anglePIDClient_("turnPID"),
                                    sidewardPIDClient_("sidewardPID") 
{
    angle_sub_ = nh.subscribe("/varun/sensors/imu/yaw", 1, &moveForward::imuAngleCB, this);
    depth_sub_ = nh.subscribe("/varun/sensors/depth", 1, &moveForward::depthCB, this);
    nh.setParam("/pwm_forward_right", pwm_);
    nh.setParam("/pwm_forward_left", pwm_);

    depthGoalReceived = false;
    angleGoalReceived = false;
}

moveForward::~moveForward() {
}

void moveForward::setActive(bool status) {
    if (status == true) {
        ROS_INFO("Waiting for sidewardPID server to start.");
        sidewardPIDClient_.waitForServer();

        ROS_INFO("sidewardPID server started, sending goal.");
        sideward_PID_goal.target_distance = 0;
        sidewardPIDClient_.sendGoal(sideward_PID_goal);

        ROS_INFO("Waiting for upwardPID server to start.");
        upwardPIDClient_.waitForServer();

        ROS_INFO("Waiting for anglePID server to start.");
        anglePIDClient_.waitForServer(); 
    }

    if (status == false) {
        sidewardPIDClient_.cancelGoal();
        upwardPIDClient_.cancelGoal();
        anglePIDClient_.cancelGoal();
    }
}   

void moveForward::setReferenceAngle(double angle_) {
    angle = angle_;
}

void moveForward::setReferenceDepth(double depth_) {
    depth = depth_;
}

void moveForward::imuAngleCB(const std_msgs::Float64Ptr &_msg) {
    angle = _msg->data;
    if (angleGoalReceived) {
        ROS_INFO("anglePID server started, sending goal.");
        angle_PID_goal.target_angle = 0;
        anglePIDClient_.sendGoal(angle_PID_goal);
    }
}

void moveForward::depthCB(const std_msgs::Float64Ptr &_msg) {
    depth = _msg->data;
    if (depthGoalReceived) {
        ROS_INFO("upwardPID server started, sending goal.");
        upward_PID_goal.target_depth = depth;
        upwardPIDClient_.sendGoal(upward_PID_goal);
        depthGoalReceived = false;
    }

}