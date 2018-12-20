#include <straight_server.h>

moveStraight::moveStraight(int pwm_) : anglePIDClient("turnPID"){
    ROS_INFO("Inside callback");
    ROS_INFO("PWM %d", pwm_);
    nh.setParam("/pwm_forward_right_straight", pwm_);
    nh.setParam("/pwm_forward_left_straight", pwm_);
    goalReceived = false;
    ROS_INFO("Params set");
    sub_ = nh.subscribe("/mavros/imu/yaw", 1, &moveStraight::imuAngleCB, this);
    //spin_thread_ = new boost::thread(boost::bind(&moveStraight::spinThread_, this));
}

moveStraight::~moveStraight() {
}

void moveStraight::setActive(bool status) {
    if (status) {
        spinThread();
        spin_thread = new boost::thread(boost::bind(&moveStraight::spinThread_, this));
    }
    else {
        if (goalReceived) {
            anglePIDClient.cancelGoal();
        }
        //spin_thread_->join();
        ROS_INFO("Forward completed");
        spin_thread->join();
        nh.setParam("/kill_signal",1);
    }
}   

void moveStraight::imuAngleCB(const std_msgs::Float32Ptr &_msg) {
    angle = _msg->data;
    goalReceived = true;
}

void moveStraight::spinThread() {

    ROS_INFO("Waiting for turnPID server to start.");
    anglePIDClient.waitForServer();
    ROS_INFO("Server waiting compelted");
    double then = ros::Time::now().toSec();
    while (!goalReceived_ref) {
        double now = ros::Time::now().toSec();
        if (now - then > 5) {
            break;
        }
    }
    if (goalReceived_ref) {
        ROS_INFO("turnPID server started, sending goal.");
        angle_PID_goal.target_angle = angle;
        anglePIDClient.sendGoal(angle_PID_goal);
        ROS_INFO("Sent the goal to client");
    }
}

void moveStraight::spinThread_() {
    ROS_INFO("Spinng the ros spin thread");
    ros::spin();
}

void moveStraight::setThrust(int _pwm) {
    nh.setParam("/pwm_forward_right_straight", _pwm);
    nh.setParam("/pwm_forward_left_straight", _pwm);
}
