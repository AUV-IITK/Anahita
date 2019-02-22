#include <marker_dropper.h>

MarkerDropper::MarkerDropper(): forwardPIDClient("forwardPID"), sidewardPIDClient("sidewardPID"), 
                                th(25), anglePIDClient("turnPID") {}
MarkerDropper::~MarkerDropper() {}

bool MarkerDropper::setActive(bool status) {

    if (status) {

        nh.setParam("/current_task", "marker_dropper_bottom");
        ROS_INFO("Current task: Marker Dropper Bottom");

        move_straight.activate (50, "local");
        depth_stabilise.activate ("reference");

        nh.setParam("/pwm_sway", 50);

        ROS_INFO("Finding Marker Dropper ....");

        if (!th.isDetected("marker_dropper_bottom", 20)) {
            ROS_INFO("Unable to detect Marker Dropper");
            // return false;
        }
        nh.setParam("/pwm_sway", 0);

        move_straight.deActivate ();
        ROS_INFO("Marker Dropper Detected");

        ROS_INFO("Waiting for forwardPID server to start.");
        forwardPIDClient.waitForServer();

        ROS_INFO("forwardPID server started, sending goal.");
        forwardPIDgoal.target_distance = 0;
        forwardPIDClient.sendGoal(forwardPIDgoal);

        ROS_INFO("Waiting for sidewardPID server to start.");
        sidewardPIDClient.waitForServer();

        ROS_INFO("sidewardPID server started, sending goal.");
        sidewardPIDGoal.target_distance = 0;
        sidewardPIDClient.sendGoal(sidewardPIDGoal);

        ////////////////////////////////////////////////////

        if (!th.isAchieved(0, 15, "forward")) {
            ROS_INFO("Marker Dropper, Forward not achieved");
            // return false;
        }
        if (!th.isAchieved(0, 15, "sideward")) {
            ROS_INFO("Marker Dropper, sideward not achieved");
            // return false;
        }

        ros::Duration(3).sleep();

        // Drop the ball

        nh.setParam("/marker_dropper", 1);
        ros::Duration(2).sleep();

        ROS_INFO("Marker Dropper Finished");
        depth_stabilise.deActivate ();

        ROS_INFO("Torpedo Finished");
        
    }
    else {
        forwardPIDClient.cancelGoal();
        sidewardPIDClient.cancelGoal();
        anglePIDClient.cancelGoal();

        ROS_INFO("Killing the thrusters");
	    nh.setParam("/kill_signal", true);

        ROS_INFO("Closing Marker Dropper");
    }
    return true;
}
