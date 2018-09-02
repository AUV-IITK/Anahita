#include <move_sideward_server.h>

moveSideward::moveSideward(int pwm_): anglePIDClient("turnPID/sensor") {
    spin_thread = new boost::thread(boost::bind(&moveSideward::spinThread, this));
    sub_ = nh.subscribe("/varun/sensors/imu/yaw", 1, &moveSideward::imuAngleCB, this);
    
    nh.setParam("/pwm_sideward_front_straight", pwm_);
    nh.setParam("/pwm_sideward_back_straight", pwm_);
}

moveSideward::~moveSideward() {
    spin_thread->join();
}

void moveSideward::setActive(bool status) {
    
    if (status == true) {
        ROS_INFO("Waiting for sidewardPID server to start.");
        anglePIDClient.waitForServer();

        ROS_INFO("sidewardPID server started, sending goal.");
        angle_PID_goal.target_angle = angle;
        anglePIDClient.sendGoal(angle_PID_goal);
    }

    if (status == false) {
        anglePIDClient.cancelGoal();
    }
}   

void moveSideward::spinThread() {
    ros::spin();
}

void moveSideward::imuAngleCB(const std_msgs::Float64Ptr &_msg) {
    angle = _msg->data;
}