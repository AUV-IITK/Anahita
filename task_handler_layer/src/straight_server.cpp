#include <straight_server.h>

moveStraight::moveStraight(int pwm_) : anglePIDClient("turnPID") { nh.setParam("/pwm_surge", pwm_); }

moveStraight::~moveStraight() {
}

void moveStraight::setActive(bool status) {
    if (status) {
        spin_thread = new boost::thread(boost::bind(&moveStraight::spinThread, this));
    }
    else {
        nh.setParam("/kill_signal", true);
        anglePIDClient.cancelGoal();
        ROS_INFO("Straight Server goal cancelled");
        spin_thread->join();
    }
}   

void moveStraight::spinThread() {

    ROS_INFO("Waiting for turnPID server to start.");
    anglePIDClient.waitForServer();
    ROS_INFO("turnPID server started, sending goal.");
    angle_PID_goal.target_angle = 0;
    anglePIDClient.sendGoal(angle_PID_goal);
}

void moveStraight::setThrust(int _pwm) {
    ros::Duration(1).sleep();
    nh.setParam("/pwm_surge", _pwm);
}
