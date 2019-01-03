#include <octagon.h>

Octagon::Octagon(): forwardPIDClient("forwardPID"), sidewardPIDClient("sidewardPID"), th(15) {}
Octagon::~Octagon() {}

void Octagon::setActive(bool status) {

    if (status) {

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

        th.isAchieved(0, 5, "forward");

        ros::Duration(3).sleep();

        ROS_INFO("Octagon Task, going up");
        nh_.setParam("/pwm_heave", 50);

        ros::Duration(6).sleep();
        
        ROS_INFO("Killing the thrusters");
	    nh_.setParam("/kill_signal", true);

    }
    else {
        forwardPIDClient.cancelGoal();
        sidewardPIDClient.cancelGoal();
        ROS_INFO("Closing Octagon");
    }
}
