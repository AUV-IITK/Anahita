#include <move_downward_server.h>

moveDownward::moveDownward(int pwm_): anglePIDClient("turnPID") {
    sub_ = nh.subscribe("/mavros/imu/yaw", 1, &moveDownward::imuAngleCB, this);
    goalReceived = false;
    
    nh.setParam("/pwm_heave", pwm_);
}

moveDownward::~moveDownward() {
}

void moveDownward::setActive(bool status) {
    
    if (status == true) {
        spin_thread_ = new boost::thread(boost::bind(&moveDownward::spinThread_, this));
    	spin_thread = new boost::thread(boost::bind(&moveDownward::spinThread, this));
    }
    else {
        if (goalReceived) {
           anglePIDClient.cancelGoal();
        }
        close_loop = true; 
        spin_thread->join();
        nh.setParam("/kill_signal", true);
        nh.setParam("/kill_signal", false);
        spin_thread_->join();
    }
}

void moveDownward::spinThread() {
    ROS_INFO("Waiting for turnPID server to start.");
    anglePIDClient.waitForServer();
    double then = ros::Time::now().toSec();
    while(!goalReceived) {
        double now = ros::Time::now().toSec();
        if (now - then > 5 || close_loop) {
            break;
        }
    }
    if (goalReceived) {
        ROS_INFO("turnPID server started, sending goal.");
        angle_PID_goal.target_angle = angle;
        anglePIDClient.sendGoal(angle_PID_goal);
    }
}

void moveDownward::spinThread_() {
    // ros::spin();
}

void moveDownward::imuAngleCB(const std_msgs::Float32Ptr &_msg) {
	angle = _msg->data;
    goalReceived = true;
}

void moveDownward::setThrust(int _pwm) {
    nh.setParam("/pwm_heave", _pwm);
}
