#include <straight_server.h>

moveStraight::moveStraight(int pwm_): anglePIDClient("turnPID") {
    nh.setParam("/pwm_forward_right", pwm_);
    nh.setParam("/pwm_forward_left", pwm_);
    goalReceived = false;
    sub_ = nh.subscribe("/mavros/imu/yaw", 1, &moveStraight::imuAngleCB, this);
    spin_thread_ = new boost::thread(boost::bind(&moveStraight::spinThread_, this));
}

moveStraight::~moveStraight() {
}

void moveStraight::setActive(bool status) {
    if (status) {
        spin_thread = new boost::thread(boost::bind(&moveStraight::spinThread, this));
    }
    else {
        anglePIDClient.cancelGoal();
        spin_thread->join();
        spin_thread_->join();
    }
}   

void moveStraight::imuAngleCB(const std_msgs::Float32Ptr &_msg) {
    std::cout << "callback" << std::endl;
    angle = _msg->data;
    goalReceived = true;
}

void moveStraight::spinThread() {

    ROS_INFO("Waiting for turnPID server to start.");
    anglePIDClient.waitForServer();

    while (!goalReceived) {
    }
    
    ROS_INFO("turnPID server started, sending goal.");
    angle_PID_goal.target_angle = angle;
    anglePIDClient.sendGoal(angle_PID_goal);
}

void moveStraight::spinThread_() {
    ros::spin();
}