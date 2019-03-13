#include <octagon.h>

Octagon::Octagon(): surgePIDClient("surgePID"), swayPIDClient("swayPID"), th(15) {}
Octagon::~Octagon() {}

bool Octagon::setActive(bool status) {

    if (status) {

        nh_.setParam("/use_local_yaw", true);
        nh_.setParam("/use_reference_yaw", false);

        ROS_INFO("Waiting for swayPID server to start, task buoy.");
        swayPIDClient.waitForServer();

        ROS_INFO("swayPID server started, sending goal, task buoy.");
        swayPIDgoal.target_sway = 0;
        swayPIDClient.sendGoal(swayPIDgoal);

        ///////////////////////////////////////////////////

        ROS_INFO("Waiting for surgePID server to start.");
        surgePIDClient.waitForServer();

        ROS_INFO("surgePID server started, sending goal.");
        surgePIDgoal.target_surge = 0;
        surgePIDClient.sendGoal(surgePIDgoal);

        ////////////////////////////////////////////////////

        if (!th.isAchieved(0, 15, "surge")) {
            ROS_INFO("Octagon, Failed to achieve surge goal");
            // return false;
        }

        if (!th.isAchieved(0, 10, "sway")) {
            ROS_INFO("Octagon, Failed to achieve sway goal");
            // return false;
        }

        ROS_INFO("Octagon Finished");
    }
    else {
        nh_.setParam("/use_local_yaw", false);
        surgePIDClient.cancelGoal();
        swayPIDClient.cancelGoal();
        ROS_INFO("Closing Octagon");
        ROS_INFO("Killing the thrusters");
	    nh_.setParam("/kill_signal", true);
    }
}
