#include <move_downward_server.h>

moveDownward::moveDownward(int pwm_): anglePIDClient("turnPID") { nh.setParam("/pwm_heave", pwm_); }

moveDownward::~moveDownward() {}

void moveDownward::setActive(bool status) {
    
    if (status == true) {
    	spin_thread = new boost::thread(boost::bind(&moveDownward::spinThread, this));
    }
    else {
        anglePIDClient.cancelGoal();
        spin_thread->join();
        nh.setParam("/kill_signal", true);
    }
}

void moveDownward::spinThread() {
    ROS_INFO("Waiting for turnPID server to start.");
    anglePIDClient.waitForServer();
    ROS_INFO("turnPID server started, sending goal.");
    angle_PID_goal.target_angle = 0;
    anglePIDClient.sendGoal(angle_PID_goal);
}

void moveDownward::setThrust(int _pwm) {
    int pub_count = 0;
    while (ros::ok() && pub_count <=10) {
        pub_count++;
        nh.setParam("/pwm_heave", _pwm);
    }
}
