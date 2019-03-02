#include <line.h>

lineTask::lineTask(): swayPIDClient("swayPID"), yawPIDClient("yawPID"), 
                    surgePIDClient("surgePID"), th(30) {}

lineTask::~lineTask() {}

bool lineTask::setActive(bool value) {
    if (value) {

        nh_.setParam("/use_local_yaw", false);
        nh_.setParam("/use_reference_yaw", false);
        nh_.setParam("/disable_imu", true);
        nh_.setParam("/enable_pressure", true);

        ROS_INFO("Waiting for swayPID server to start.");
        swayPIDClient.waitForServer();

        ROS_INFO("swayPID server started, sending goal.");
        sway_PID_goal.target_sway = 0;
        swayPIDClient.sendGoal(sway_PID_goal);

        ROS_INFO("Waiting for surgePID server to start.");
        surgePIDClient.waitForServer();

        ROS_INFO("surgePID server started, sending goal.");
        surge_PID_goal.target_surge = 0;
        surgePIDClient.sendGoal(surge_PID_goal);

        if (!th.isAchieved(0, 25, "surge")) {
            ROS_INFO("NOT able to align to the center of the line");
            // return false;
        }

        ROS_INFO("Waiting for yawPID server to start.");
        yawPIDClient.waitForServer();

        ROS_INFO("yawPID server started, sending goal.");
        
        yaw_PID_goal.target_yaw = 0;
        yawPIDClient.sendGoal(yaw_PID_goal);

        if (!th.isAchieved(0, 2, "yaw")) {
            ROS_INFO("Bot Unbale to align with the line");
            // return false;
        }
    	nh_.setParam("/set_local_yaw", true);

        ROS_INFO("Line Task Finished");
    }
    else {
        yawPIDClient.cancelGoal();
        swayPIDClient.cancelGoal();
        surgePIDClient.cancelGoal();
        
        nh_.setParam("/disable_imu", false);
        nh_.setParam("/kill_signal", true);
    }
    return true;
}
