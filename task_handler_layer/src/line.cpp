#include <line.h>

lineTask::lineTask(): sidewardPIDClient("sidewardPID"), anglePIDClient("turnPID"), 
                    forwardPIDClient("forwardPID"), th(30) {
    sub_ = nh_.subscribe("/mavros/imu/yaw", 1, &lineTask::angleCB, this);
}

lineTask::~lineTask() {}

bool lineTask::setActive(bool value) {
    if (value) {
        nh_.setParam("/use_local_yaw", false);
        nh_.setParam("/use_reference_yaw", false);
        nh_.setParam("/disable_imu", true);

        ROS_INFO("Waiting for sidewardPID server to start.");
        sidewardPIDClient.waitForServer();

        ROS_INFO("sidewardPID server started, sending goal.");
        sideward_PID_goal.target_distance = 0;
        sidewardPIDClient.sendGoal(sideward_PID_goal);

        ROS_INFO("Waiting for forwardPID server to start.");
        forwardPIDClient.waitForServer();

        ROS_INFO("forwardPID server started, sending goal.");
        forward_PID_goal.target_distance = 0;
        forwardPIDClient.sendGoal(forward_PID_goal);

        if (!th.isAchieved(0, 10, "forward")) {
            ROS_INFO("NOT able to align to the center of the line");
            return false;
        }

        ROS_INFO("Waiting for anglePID server to start.");
        anglePIDClient.waitForServer();

        bool temp = false;

        while (ros::ok()) {
            mtx.lock();
            temp = angleReceived;
            mtx.unlock();
            if (temp) {
                break;
            }
        }

        ROS_INFO("anglePID server started, sending goal.");
        
        mtx.lock();
        double angle = angle_;
        mtx.unlock();

        angle_PID_goal.target_angle = 0;
        anglePIDClient.sendGoal(angle_PID_goal);

        if (!th.isAchieved(0, 5, "angle")) {
            ROS_INFO("Bot Unbale to align with the line");
            return false;
        }
    }
    else {
        anglePIDClient.cancelGoal();
        sidewardPIDClient.cancelGoal();
        forwardPIDClient.cancelGoal();
        
        nh_.setParam("/disable_imu", false);
        nh_.setParam("/kill_signal", true);
    }
    return true;
}

void lineTask::angleCB(const std_msgs::Float32ConstPtr& _msg) {
    mtx.lock();
    angle_ = _msg->data;
    angleReceived = true;
    mtx.unlock();
}