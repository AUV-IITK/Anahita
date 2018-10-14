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
        spin_thread = new boost::thread(boost::bind(&singleBuoy::spinThread, this));
    }

    else {
        anglePIDClient.cancelGoal();
        spin_thread->join();
    }
}

void moveSideward::spinThread() {
    ROS_INFO("Waiting for turnPID server to start.");
    anglePIDClient.waitForServer();
    while(!goalReceived) {} 
    ROS_INFO("turnPID server started, sending goal.");
    angle_PID_goal.target_angle = angle;
    anglePIDClient.sendGoal(angle_PID_goal);
}

void moveSideward::imuAngleCB(const std_msgs::Float64Ptr &_msg) {
	angle = _msg->data;
    goalReceived = true;
}
