#include <straight_server.h>

moveStraight::moveStraight(int pwm_): anglePIDClient("turnPID") {
    //spin_thread = new boost::thread(boost::bind(&moveStraight::spinThread, this));
     
    //std::cout << "straight server pwm set to: " << pwm_ << std::endl;
    nh.setParam("/pwm_forward_right", pwm_);
    nh.setParam("/pwm_forward_left", pwm_);
    flag = false;

    sub_ = nh.subscribe("/varun/sensors/imu/yaw", 1, &moveStraight::imuAngleCB, this);
}

moveStraight::~moveStraight() {
    spin_thread->join();
}

void moveStraight::setActive(bool status) {
    if (status == true) {
        ROS_INFO("Waiting for anglePID server to start.");
        anglePIDClient.waitForServer();
        ROS_INFO("anglePID server started, sending goal.");
        flag = true;
   }

    if (status == false) {
        anglePIDClient.cancelGoal();
    }
}   

void moveStraight::spinThread() {
    ros::spin();
}

void moveStraight::imuAngleCB(const std_msgs::Float32Ptr &_msg) {
    //std::cout << "called back move straight" << std::endl;
    angle = _msg->data;
    if (flag) {
        angle_PID_goal.target_angle = angle;
        //std::cout << "straight server goal set to: " << angle << std::endl;
        anglePIDClient.sendGoal(angle_PID_goal);
        flag = false;
    }
}
