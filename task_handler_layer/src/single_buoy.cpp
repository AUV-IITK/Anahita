#include <single_buoy.h>

singleBuoy::singleBuoy(): surgePIDClient("surgePID"), swayPIDClient("swayPID"), 
                        yawPIDClient("yawPID"), heavePIDClient("heavePID"), th(30) {
    sub_ = nh_.subscribe("/anahita/x_coordinate", 1, &singleBuoy::callback, this);
}
singleBuoy::~singleBuoy() {}

bool singleBuoy::setActive(bool status) {

    if (status) {

        ros::Duration(1.5).sleep();
        nh_.setParam("/enable_pressure", false);

        ROS_INFO("Waiting for swayPID server to start, task buoy.");
        swayPIDClient.waitForServer();

        swayPIDgoal.target_sway = 0;
        swayPIDClient.sendGoal(swayPIDgoal);

        ///////////////////////////////////////////////////

        ROS_INFO("Waiting for heavePID server to start.");
        heavePIDClient.waitForServer();

        heavePIDgoal.target_heave = 0; // for gazebo
        heavePIDClient.sendGoal(heavePIDgoal);

        ///////////////////////////////////////////////////

        ROS_INFO("Waiting for yawPID server to start.");
        yawPIDClient.waitForServer();

        yawPIDGoal.target_yaw = 0;
        yawPIDClient.sendGoal(yawPIDGoal);

        if (!th.isAchieved(0, 40, "sway")) {
            ROS_INFO("Unable to achieve sway goal");
        }

        /////////////////////////////////////////////////////

        nh_.setParam("/pwm_surge", 50);

        while (ros::ok()) { if (goal_received) { break; } }

        while (ros::ok()) {
            if (forward_distance <= 125) { break; }
        }

        ROS_INFO("Waiting for surgePID server to start.");
        surgePIDClient.waitForServer();

        surgePIDgoal.target_surge = 50;
        surgePIDClient.sendGoal(surgePIDgoal);

        if (!th.isAchieved(50, 10, "surge")) {
            ROS_INFO("Unable to achieve surge goal");
        }

        ROS_INFO("forward distance equal 50");
    	
    	// if (!th.isAchieved(0, 35, "sway")) {
        //     ROS_INFO("Unable to achieve sway goal");
        // }

        ROS_INFO("Hitting the buoy now!");

        depthStabilise depth_stabilise;
        depth_stabilise.activate("current");

        surgePIDClient.cancelGoal();
        swayPIDClient.cancelGoal();
    	heavePIDClient.cancelGoal();

        nh_.setParam("/kill_signal", true);
        ros::Duration(0.5).sleep();

        nh_.setParam("/pwm_surge", 50);

        ros::Duration(7).sleep();
        //////////////////////////////////////////////////////
        nh_.setParam("/pwm_surge", 0);
        ros::Duration(0.5).sleep();
        nh_.setParam("/pwm_surge", -50);

        ROS_INFO("Buoy Task, moving backward");
        
        ros::Duration(5).sleep(); 
        
        nh_.setParam("/pwm_surge", 0);

        ROS_INFO("moving backward finished");

        if (!th.isAchieved(0, 2, "yaw")) {
            ROS_INFO("Unable to achieve yaw goal");
            return false;
        }

        ROS_INFO("Buoy Task Finished!");
    }
    else {
        yawPIDClient.cancelGoal();

        ROS_INFO("Closing Single Buoy");
        ROS_INFO("Killing the thrusters");
	    
        nh_.setParam("/use_reference_yaw", false);
        nh_.setParam("/kill_signal", true);
    }
    return true;
}

void singleBuoy::callback (const std_msgs::Float32Ptr& _msg) {
    forward_distance = _msg->data;
    goal_received = true;
}