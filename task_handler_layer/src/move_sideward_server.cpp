#include <move_sideward_server.h>

moveSideward::moveSideward(int pwm_): anglePIDClient("turnPID") {
    sub_ = nh.subscribe("/mavros/imu/yaw", 1, &moveSideward::imuAngleCB, this);
    goalReceived = false;
    
    nh.setParam("/pwm_sideward_front_straight", pwm_);
    nh.setParam("/pwm_sideward_back_straight", pwm_);
    //spin_thread_ = new boost::thread(boost::bind(&moveSideward::spinThread_, this));
}

moveSideward::~moveSideward() {
}

void moveSideward::setActive(bool status) {
    
    if (status == true) {
        spinThread();
	spin_thread = new boost::thread(boost::bind(&moveSideward::spinThread_, this));
    }
    else {
        if (goalReceived) {
           anglePIDClient.cancelGoal();
        }
        spin_thread->join();
        nh.setParam("/kill_signal", 1);
    //    spin_thread->join();
       // spin_thread_->join();
    }
}

void moveSideward::spinThread() {
    ROS_INFO("Waiting for turnPID server to start.");
    anglePIDClient.waitForServer();
    double then = ros::Time::now().toSec();
    while(!goalReceived) {
        double now = ros::Time::now().toSec();
        if (now - then > 5) {
            std::cout<<"Hello"<<std::endl;
            break;
        }
    }
    if (goalReceived) {
        ROS_INFO("turnPID server started, sending goal.");
        angle_PID_goal.target_angle = angle;
        anglePIDClient.sendGoal(angle_PID_goal);
    }
}

void moveSideward::spinThread_() {
    ROS_INFO("Spinning the ros spin thread");
    ros::spin();
}

void moveSideward::imuAngleCB(const std_msgs::Float32Ptr &_msg) {
	angle = _msg->data;
    goalReceived = true;
}

void moveSideward::setThrust(int _pwm) {
    nh.setParam("/pwm_sideward_front_straight", _pwm);
    nh.setParam("/pwm_sideward_back_straight", _pwm);
}
