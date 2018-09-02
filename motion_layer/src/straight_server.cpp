#include <straight_server.h>

moveStraight::moveStraight(int pwm_): anglePIDClient("turnPID") {
    spin_thread = new boost::thread(boost::bind(&moveStraight::spinThread, this));

    nh.setParam("/pwm_forward_right", pwm_);
    nh.setParam("/pwm_forward_left", pwm_);

    sub_ = nh.subscribe("/varun/sensors/yaw", 1, &moveStraight::imuAngleCB, this);
}

moveStraight::~moveStraight() {
    spin_thread->join();
}

void moveStraight::setActive(bool status) {
    if (status == true) {
        ROS_INFO("Waiting for anglePID server to start.");
        anglePIDClient.waitForServer();

        ROS_INFO("anglePID server started, sending goal.");
        angle_PID_goal.target_angle = angle;
        anglePIDClient.sendGoal(angle_PID_goal);
    }

    if (status == false) {
        anglePIDClient.cancelGoal();
    }
}   

void moveStraight::spinThread() {
    ros::spin();
}

void moveStraight::imuAngleCB(const std_msgs::Float64Ptr &_msg) {
    angle = _msg->data;
}