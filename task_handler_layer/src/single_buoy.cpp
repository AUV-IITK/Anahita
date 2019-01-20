#include <single_buoy.h>

singleBuoy::singleBuoy(): forwardPIDClient("forwardPID"), sidewardPIDClient("sidewardPID"), 
                        anglePIDClient("turnPID"), upwardPIDClient("upwardPID"), th(30) {}
singleBuoy::~singleBuoy() {}

bool singleBuoy::setActive(bool status) {

    if (status) {

        ROS_INFO("Waiting for sidewardPID server to start, task buoy.");
        sidewardPIDClient.waitForServer();

        ROS_INFO("sidewardPID server started, sending goal, task buoy.");
        sidewardPIDgoal.target_distance = 0;
        sidewardPIDClient.sendGoal(sidewardPIDgoal);

        ///////////////////////////////////////////////////

        ROS_INFO("Waiting for upwardPID server to start.");
        upwardPIDClient.waitForServer();

        ROS_INFO("upwardPID server started, sending goal.");
        upwardPIDgoal.target_depth = 0; // for gazebo
        upwardPIDClient.sendGoal(upwardPIDgoal);

        ///////////////////////////////////////////////////

        ROS_INFO("Waiting for anglePID server to start.");
        anglePIDClient.waitForServer();

        ROS_INFO("anglePID server started, sending goal.");

        anglePIDGoal.target_angle = 0;
        anglePIDClient.sendGoal(anglePIDGoal);

        if (!th.isAchieved(0, 40, "sideward")) {
            ROS_INFO("Unable to achieve sideward goal");
        }

        /////////////////////////////////////////////////////

        ROS_INFO("Waiting for forwardPID server to start.");
        forwardPIDClient.waitForServer();

        ROS_INFO("forward distance received");

        forwardPIDgoal.target_distance = 50;
        forwardPIDClient.sendGoal(forwardPIDgoal);

        if (!th.isAchieved(50, 10, "forward")) {
            ROS_INFO("Unable to achieve forward goal");
        }

        ROS_INFO("forward distance equal 50");
    	
    	if (!th.isAchieved(0, 35, "sideward")) {
            ROS_INFO("Unable to achieve sideward goal");
        }

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

        // if (th.isDetected("red_buoy", 7)) {

    	    // ROS_INFO("detected the buoy after hitting");
            // ROS_INFO("SidewardPID Client sending goal again, task buoy.");
            // sidewardPIDgoal.target_distance = 0;
            // sidewardPIDClient.sendGoal(sidewardPIDgoal);
            
            // ROS_INFO("UpwardPID Client sending goal again, task buoy.");
            // upwardPIDgoal.target_depth = 0;
            // upwardPIDClient.sendGoal(upwardPIDgoal);

            //////////////////////////////////////////////////////

            // ROS_INFO("ForwardPID Client sending goal again, task buoy.");        
            // forwardPIDgoal.target_distance = 60;
            // forwardPIDClient.sendGoal(forwardPIDgoal);

            // if (!th.isAchieved(60, 15, "forward")) {
            //     ROS_INFO("Unable to achieve forward goal");
            //     return false;
            // }
            // forwardPIDClient.cancelGoal();
        // }

        // sidewardPIDClient.cancelGoal();

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
