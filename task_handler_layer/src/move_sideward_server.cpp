#include <move_sideward_server.h>

moveSideward::moveSideward(int pwm_): anglePIDClient("turnPID") {
    sub_ = nh.subscribe("/varun/sensors/imu/yaw", 1, &moveSideward::imuAngleCB, this);
    goalReceived = false;
    
    nh.setParam("/pwm_sideward_front_straight", pwm_);
    nh.setParam("/pwm_sideward_back_straight", pwm_);
}

moveSideward::~moveSideward() {
}

void moveSideward::setActive(bool status) {
    
    if (status == true) {
        ROS_INFO("Waiting for turnPID server to start.");
        anglePIDClient.waitForServer();
	    goalReceived = true;
        ROS_INFO("turnPID server started, sending goal.");
    }

    if (status == false) {
        anglePIDClient.cancelGoal();
    }
}   

void moveSideward::imuAngleCB(const std_msgs::Float64Ptr &_msg) {
	angle = _msg->data;
	if (goalReceived) {
        angle_PID_goal.target_angle = angle;
        anglePIDClient.sendGoal(angle_PID_goal);
		goalReceived = false;	
	}
}
