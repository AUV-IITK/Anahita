#include <straight_server.h>

moveStraight::moveStraight(int pwm_): anglePIDClient("turnPID") {
    nh.setParam("/pwm_forward_right", pwm_);
    nh.setParam("/pwm_forward_left", pwm_);
    goalReceived = false;
    sub_ = nh.subscribe("/varun/sensors/imu/yaw", 1, &moveStraight::imuAngleCB, this);
}

moveStraight::~moveStraight() {
}

void moveStraight::setActive(bool status) {
    if (status == true) {
        ROS_INFO("Waiting for turnPID server to start.");
        anglePIDClient.waitForServer();
        ROS_INFO("turnPID server started, sending goal.");
        goalReceived = true;
    }

    if (status == false) {
        anglePIDClient.cancelGoal();
    }
}   

void moveStraight::imuAngleCB(const std_msgs::Float32Ptr &_msg) {
    angle = _msg->data;
    if (goalReceived) {
        angle_PID_goal.target_angle = angle;
        anglePIDClient.sendGoal(angle_PID_goal);
        goalReceived = false;
    }
}
