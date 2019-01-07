#include <straight_server.h>

moveStraight::moveStraight(int pwm_) : anglePIDClient("turnPID") {
    nh.setParam("/pwm_surge", pwm_);
    goalReceived = false;
    sub_ = nh.subscribe("/mavros/imu/yaw", 1, &moveStraight::imuAngleCB, this);
}

moveStraight::~moveStraight() {
}

void moveStraight::setActive(bool status) {
    if (status) {
        spin_thread = new boost::thread(boost::bind(&moveStraight::spinThread, this));
    }
    else {
        // if (goalReceived) {
            anglePIDClient.cancelGoal();
        // }
        // close_loop = true;
        ROS_INFO("Straight Server goal cancelled");
        spin_thread->join();
        nh.setParam("/kill_signal", true);
    }
}   

void moveStraight::imuAngleCB(const std_msgs::Float32Ptr &_msg) {
    angle = _msg->data;
    goalReceived = true;
}

void moveStraight::spinThread() {

    ROS_INFO("Waiting for turnPID server to start.");
    anglePIDClient.waitForServer();
    // double then = ros::Time::now().toSec();
    // while (!goalReceived_ref) {
    //     double now = ros::Time::now().toSec();
    //     if (now - then > 5 || close_loop) {
    //         break;
    //     }
    // }
    // if (goalReceived_ref) { 
        ROS_INFO("turnPID server started, sending goal.");
        angle_PID_goal.target_angle = 0;
        anglePIDClient.sendGoal(angle_PID_goal);
        ROS_INFO("Sent the goal to client");
    // }
}

void moveStraight::setThrust(int _pwm) {
    int pub_count = 0;
    while (ros::ok() && pub_count <= 10) {
        nh.setParam("/pwm_surge", _pwm);
        pub_count++;
    }
}
