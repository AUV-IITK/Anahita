#include <move_downward_server.h>

moveDownward::moveDownward(): yawPIDClient("yawPID") {}

moveDownward::~moveDownward() {}

void moveDownward::activate (int pwm, std::string type) {
    ros::Duration(1).sleep();
    nh_.setParam("/pwm_heave", pwm);

    if (type == "reference") { nh_.setParam("/use_reference_yaw", true); }
    else if (type == "local") { nh_.setParam("/use_local_yaw", true); }
    spin_thread = new boost::thread(boost::bind(&moveDownward::spinThread, this));
}

void moveDownward::deActivate () {
    nh_.setParam("/use_reference_yaw", false);
    nh_.setParam("/use_local_yaw", false);
    yawPIDClient.cancelGoal();
    spin_thread->join();
    nh_.setParam("/kill_signal", true);
}

void moveDownward::spinThread() {
    ROS_INFO("Waiting for turnPID server to start.");
    yawPIDClient.waitForServer();
    ROS_INFO("turnPID server started, sending goal.");
    yaw_PID_goal.target_yaw = 0;
    yawPIDClient.sendGoal(yaw_PID_goal);
}

void moveDownward::setThrust(int _pwm) {
    ros::Duration(1).sleep();
    nh_.setParam("/pwm_heave", _pwm);
}
