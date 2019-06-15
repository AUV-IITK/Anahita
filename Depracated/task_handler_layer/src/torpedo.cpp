#include <torpedo.h>

Torpedo::Torpedo(): surgePIDClient("surgePID"), swayPIDClient("swayPID"), 
                        yawPIDClient("yawPID"), heavePIDClient("heavePID"), th(120) {}

Torpedo::~Torpedo() {}

bool Torpedo::setActive(bool status) {

    if (status) {

        nh_.setParam("/use_local_yaw", true);
        nh_.setParam("/use_reference_yaw", false);
        nh_.setParam("/enable_pressure", false);

        ROS_INFO("Waiting for swayPID server to start, task torpedo.");
        swayPIDClient.waitForServer();

        ROS_INFO("swayPID server started, sending goal, task torpedo.");
        swayPIDgoal.target_sway = 0;
        swayPIDClient.sendGoal(swayPIDgoal);

        ///////////////////////////////////////////////////

        ROS_INFO("Waiting for heavePID server to start, task torpedo.");
        heavePIDClient.waitForServer();

        ROS_INFO("heavePID server started, sending goal, task torpedo.");
        heavePIDgoal.target_heave = 0; // for gazebo
        heavePIDClient.sendGoal(heavePIDgoal);

        ///////////////////////////////////////////////////

        ROS_INFO("Waiting for yawPID server to start, task torpedo.");
        yawPIDClient.waitForServer();

        ROS_INFO("yawPID server started, sending goal, task torpedo.");

        yawPIDGoal.target_yaw = 0;
        yawPIDClient.sendGoal(yawPIDGoal);

        /////////////////////////////////////////////////////

        ROS_INFO("surgePID Client sending goal, task torpedo.");        
        surgePIDgoal.target_surge = 12;
        surgePIDClient.sendGoal(surgePIDgoal);

        if (!th.isAchieved(12, 2, "surge")) {
            ROS_INFO("Unable to achieve surge in time limit");
            nh_.setParam("/kill_signal", true);		
            // return false;	
	    }	
        ROS_INFO("Firing torpedo");

        ros::Duration(2).sleep();
	
        // fire the torpedo
        nh_.setParam("/torpedo", 1);
        ros::Duration(1).sleep();
        nh_.setParam("/torpedo", 0);

        surgePIDClient.cancelGoal();
        ROS_INFO("Killing the thrusters");
	    nh_.setParam("/kill_signal", true);

        ROS_INFO("surgePID Client sending goal again, task torpedo.");        
        surgePIDgoal.target_surge = 25;
        surgePIDClient.sendGoal(surgePIDgoal);

        if (!th.isAchieved(25, 4, "surge")) {
            ROS_INFO("Unable to achieve surge in time limit");
            nh_.setParam("/kill_signal", true);
            // return false;	
	    }
        ROS_INFO("Torpedo Finished");
    }
    else {

        nh_.setParam("/use_local_yaw", false);
        
        surgePIDClient.cancelGoal();
        heavePIDClient.cancelGoal();
        swayPIDClient.cancelGoal();
        yawPIDClient.cancelGoal();
        ROS_INFO("Closing Torpedo");
    }

    return true;
}
