#include <single_buoy.h>

singleBuoy::singleBuoy(): forwardPIDClient("forwardPID"), sidewardPIDClient("sidewardPID"), 
                        anglePIDClient("turnPID"), upwardPIDClient("upwardPID"), th(30) {
    sub_ = nh_.subscribe("/anahita/x_coordinate", 1, &singleBuoy::callback, this);
}
singleBuoy::~singleBuoy() {}

bool singleBuoy::setActive(bool status) {

    if (status) {

        ros::Duration(1.5).sleep();
        nh_.setParam("/enable_pressure", false);

        ROS_INFO("Waiting for sidewardPID server to start, task buoy.");
        sidewardPIDClient.waitForServer();

        sidewardPIDgoal.target_distance = 0;
        sidewardPIDClient.sendGoal(sidewardPIDgoal);

        ///////////////////////////////////////////////////

        ROS_INFO("Waiting for upwardPID server to start.");
        upwardPIDClient.waitForServer();

        upwardPIDgoal.target_depth = 0; // for gazebo
        upwardPIDClient.sendGoal(upwardPIDgoal);

        ///////////////////////////////////////////////////

        ROS_INFO("Waiting for anglePID server to start.");
        anglePIDClient.waitForServer();

        anglePIDGoal.target_angle = 0;
        anglePIDClient.sendGoal(anglePIDGoal);

        if (!th.isAchieved(0, 40, "sideward")) {
            ROS_INFO("Unable to achieve sideward goal");
        }

        /////////////////////////////////////////////////////

        nh_.setParam("/pwm_surge", 50);

        while (ros::ok()) { if (goal_received) { break; } }

        while (ros::ok()) {
            if (forward_distance <= 125) { break; }
        }

        ROS_INFO("Waiting for forwardPID server to start.");
        forwardPIDClient.waitForServer();

        forwardPIDgoal.target_distance = 50;
        forwardPIDClient.sendGoal(forwardPIDgoal);

        if (!th.isAchieved(50, 10, "forward")) {
            ROS_INFO("Unable to achieve forward goal");
        }

        ROS_INFO("forward distance equal 50");
    	
    	// if (!th.isAchieved(0, 35, "sideward")) {
        //     ROS_INFO("Unable to achieve sideward goal");
        // }

        ROS_INFO("Hitting the buoy now!");

        depthStabilise depth_stabilise;
        depth_stabilise.setActive(true, "current");

        forwardPIDClient.cancelGoal();
        sidewardPIDClient.cancelGoal();
    	upwardPIDClient.cancelGoal();

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

        if (!th.isAchieved(0, 2, "angle")) {
            ROS_INFO("Unable to achieve angle goal");
            return false;
        }

        ROS_INFO("Buoy Task Finished!");
    }
    else {
        anglePIDClient.cancelGoal();

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