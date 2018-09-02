#include <line.h>

lineTask::lineTask(): move_straight_(100), sidewardPIDClient("sidewardPID"), anglePIDClient("turnPID/vision"), move_forward_(150) {
    spin_thread = new boost::thread(boost::bind(&lineTask::spinThread, this));
    move_forward_.setDataSource("VISION", "SENSOR");
}

lineTask::~lineTask() {

}

void lineTask::setActive(bool value) {
    if (value) {
        move_straight_.setActive(true);
        bool line_detected_signal_ = false;
        bool target_acheived_ = false;
        while(!line_detected_signal_) {
            continue;
        }
        move_straight_.setActive(false);

        ROS_INFO("Waiting for sidewardPID server to start.");
        sidewardPIDClient.waitForServer();

        ROS_INFO("sidewardPID server started, sending goal.");
        sideward_PID_goal.target_distance = 0;
        sidewardPIDClient.sendGoal(sideward_PID_goal);

        ROS_INFO("Waiting for anglePID server to start.");
        anglePIDClient.waitForServer();

        ROS_INFO("anglePID server started, sending goal.");
        angle_PID_goal.target_angle = 0;
        anglePIDClient.sendGoal(angle_PID_goal);

        while(!target_acheived_) {
            continue;
        }
        anglePIDClient.cancelGoal();
        sidewardPIDClient.cancelGoal();
        move_forward_.setActive(true);
    }
    else {
        move_straight_.setActive(false);
        anglePIDClient.cancelGoal();
        sidewardPIDClient.cancelGoal();
        move_forward_.setActive(false);
    }
}

void lineTask::spinThread() {
    ros::spin();
}