#include <octagon.h>

Octagon::Octagon(): forwardPIDClient("forwardPID"), sidewardPIDClient("sidewardPID"), th(15) {}
Octagon::~Octagon() {}

bool Octagon::setActive(bool status) {

    if (status) {

        nh_.setParam("/use_local_yaw", true);
        nh_.setParam("/use_reference_yaw", false);

        ROS_INFO("Waiting for sidewardPID server to start, task buoy.");
        sidewardPIDClient.waitForServer();

        ROS_INFO("sidewardPID server started, sending goal, task buoy.");
        sidewardPIDgoal.target_distance = 0;
        sidewardPIDClient.sendGoal(sidewardPIDgoal);

        ///////////////////////////////////////////////////

        ROS_INFO("Waiting for forwardPID server to start.");
        forwardPIDClient.waitForServer();

        ROS_INFO("forwardPID server started, sending goal.");
        forwardPIDgoal.target_distance = 0;
        forwardPIDClient.sendGoal(forwardPIDgoal);

        ////////////////////////////////////////////////////

        if (!th.isAchieved(0, 15, "forward")) {
            ROS_INFO("Octagon, Failed to achieve forward goal");
            // return false;
        }

        if (!th.isAchieved(0, 10, "sideward")) {
            ROS_INFO("Octagon, Failed to achieve sideward goal");
            // return false;
        }

        ROS_INFO("Octagon Finished");
    }
    else {
        nh_.setParam("/use_local_yaw", false);
        forwardPIDClient.cancelGoal();
        sidewardPIDClient.cancelGoal();
        ROS_INFO("Closing Octagon");
        ROS_INFO("Killing the thrusters");
	    nh_.setParam("/kill_signal", true);
    }
}
