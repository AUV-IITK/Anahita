#include <move_sideward_server.h>

moveSideward::moveSideward(int pwm_): anglePIDClient("turnPID") { nh.setParam("/pwm_sway", pwm_); }

moveSideward::~moveSideward() {}

void moveSideward::setActive(bool status) {
    
    if (status == true) {
    	spin_thread = new boost::thread(boost::bind(&moveSideward::spinThread, this));
    }
    else {
        anglePIDClient.cancelGoal();
        spin_thread->join();
        nh.setParam("/kill_signal", true);
    }
}

void moveSideward::spinThread() {
    ROS_INFO("Waiting for turnPID server to start.");
    anglePIDClient.waitForServer();
    ROS_INFO("turnPID server started, sending goal.");
    angle_PID_goal.target_angle = 0;
    anglePIDClient.sendGoal(angle_PID_goal);
}

void moveSideward::setThrust(int _pwm) {
    int pub_count = 0;
    while (ros::ok() && pub_count <= 10) {
        nh.setParam("/pwm_sway", _pwm);
        pub_count++;
    }
}
