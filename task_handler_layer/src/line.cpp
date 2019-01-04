#include <line.h>

lineTask::lineTask(): sidewardPIDClient("sidewardPID"), anglePIDClient("turnPID"), 
                    forwardPIDClient("forwardPID"), th(15) {
    sub_ = nh_.subscribe("/mavros/imu/data", 1, &lineTask::angleCB, this);
}

lineTask::~lineTask() {}

bool lineTask::setActive(bool value) {
    if (value) {
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

        if (!th.isAchieved(0, 5, "forward")) {
            ROS_INFO("NOT able to align to the center of the line");
            return false;
        }

        ROS_INFO("Waiting for anglePID server to start.");
        anglePIDClient.waitForServer();

        while (ros::ok() && !angleReceived) { continue; }

        ROS_INFO("anglePID server started, sending goal.");
        angle_PID_goal.target_angle = -angle_;
        anglePIDClient.sendGoal(angle_PID_goal);

        th.isAchieved(0, 5, "angle");
    }
    else {
        anglePIDClient.cancelGoal();
        sidewardPIDClient.cancelGoal();
        forwardPIDClient.cancelGoal();
    }
    return true;
}

void lineTask::angleCB(const std_msgs::Float32ConstPtr& _msg) {
    angle_ = _msg->data;
    angleReceived = true;
}