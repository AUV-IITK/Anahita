#include <marker_dropper.h>

MarkerDropper::MarkerDropper(): surgePIDClient("surgePID"), swayPIDClient("swayPID"), 
                                th(25), yawPIDClient("yawPID") {}
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

        ROS_INFO("Waiting for surgePID server to start.");
        surgePIDClient.waitForServer();

        ROS_INFO("surgePID server started, sending goal.");
        surgePIDgoal.target_surge = 0;
        surgePIDClient.sendGoal(surgePIDgoal);

        ROS_INFO("Waiting for swayPID server to start.");
        swayPIDClient.waitForServer();

        ROS_INFO("swayPID server started, sending goal.");
        swayPIDGoal.target_sway = 0;
        swayPIDClient.sendGoal(swayPIDGoal);

        ////////////////////////////////////////////////////

        if (!th.isAchieved(0, 15, "surge")) {
            ROS_INFO("Marker Dropper, surge not achieved");
            // return false;
        }
        if (!th.isAchieved(0, 15, "sway")) {
            ROS_INFO("Marker Dropper, sway not achieved");
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
        surgePIDClient.cancelGoal();
        swayPIDClient.cancelGoal();
        yawPIDClient.cancelGoal();

        ROS_INFO("Killing the thrusters");
	    nh.setParam("/kill_signal", true);

        ROS_INFO("Closing Marker Dropper");
    }
    return true;
}
