#include <move_sideward_server.h>

moveSideward::moveSideward(): anglePIDClient("turnPID") {}

moveSideward::~moveSideward() {}

void moveSideward::activate (int pwm, std::string type) {
    ros::Duration(1).sleep();
    nh_.setParam("/pwm_sway", pwm);

    if (type == "reference") { nh_.setParam("/use_reference_yaw", true); }
    else if (type == "local") { nh_.setParam("/use_local_yaw", true); }
    spin_thread = new boost::thread(boost::bind(&moveSideward::spinThread, this));
}

void moveSideward::deActivate () {
    nh_.setParam("/use_reference_yaw", false);
    nh_.setParam("/use_local_yaw", false);
    anglePIDClient.cancelGoal();
    spin_thread->join();
    nh_.setParam("/kill_signal", true);
}

void moveSideward::spinThread() {
    ROS_INFO("Waiting for turnPID server to start.");
    anglePIDClient.waitForServer();
    ROS_INFO("turnPID server started, sending goal.");
    angle_PID_goal.target_angle = 0;
    anglePIDClient.sendGoal(angle_PID_goal);
}

void moveSideward::setThrust(int _pwm) {
    ros::Duration(1).sleep();
    nh_.setParam("/pwm_sway", _pwm);
}
