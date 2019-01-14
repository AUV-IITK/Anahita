#include <marker_dropper.h>

MarkerDropper::MarkerDropper(): forwardPIDClient("forwardPID"), sidewardPIDClient("sidewardPID"), 
                                th(15), move_straight(0), anglePIDClient("turnPID") 
{
    forward_sub_ = nh.subscribe("/anahita/x_coordinate", 1, &MarkerDropper::forwardCB, this);
}
MarkerDropper::~MarkerDropper() {}

bool MarkerDropper::setActive(bool status) {

    if (status) {

        nh.setParam("/pwm_surge", 50);
        nh.setParam("/use_reference_yaw", false);
        nh.setParam("/use_lcoal_yaw", false);

        ROS_INFO("Waiting for sidewardPID server to start.");
        sidewardPIDClient.waitForServer();

        ROS_INFO("sidewardPID server started, sending goal.");

        sidewardPIDGoal.target_distance = 0;
        sidewardPIDClient.sendGoal(sidewardPIDGoal);

        anglePIDClient.waitForServer();

        anglePIDGoal.target_angle = 0;
        anglePIDClient.sendGoal(anglePIDGoal);

        while (ros::ok()) {
            // mtx.lock();
            bool temp = forwardGoalReceived;
            // mtx.unlock();
            if (temp) { break; }
        }

        while (ros::ok()) {
            // mtx.lock();
            double data = forward_distance_;
            // mtx.unlock();
            if (data <= 200) { break; }
        }
        sidewardPIDClient.cancelGoal();

        nh.setParam("/current_task", "line");
        ROS_INFO("Current task: Line");

        if (!th.isDetected("line", 20)) {
            ROS_INFO("Line Not detected");
            // move_straight.setActive(false);
            return false;
        }
        anglePIDClient.cancelGoal();

        lineTask line;

        if (!line.setActive(true)) {
            ROS_INFO("Unable to perform line");
            line.setActive(false);
            return false;
        }
        line.setActive(false);

        ROS_INFO("Aligned to the line");

        nh.setParam("/set_local_yaw", true);

        ///////////////////////////////////////////////////

        nh.setParam("/current_task", "marker_dropper_bottom");
        ROS_INFO("Current task: Marker Dropper Bottom");

        move_straight.setThrust(50);
        move_straight.setActive(true, "local");

        ROS_INFO("Finding Marker Dropper ....");

        if (!th.isDetected("marker_dropper_bottom", 20)) {
            ROS_INFO("Unable to detect Marker Dropper");
            move_straight.setActive(false, "local");
            return false;
        }
        move_straight.setActive(false, "local");
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
            return false;
        }
        if (!th.isAchieved(0, 15, "sideward")) {
            ROS_INFO("Marker Dropper, sideward not achieved");
            return false;
        }

        ros::Duration(3).sleep();

        // Drop the ball
        
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

void MarkerDropper::forwardCB (const std_msgs::Float32ConstPtr &_msg) {
    // mtx.lock();
    forward_distance_ = _msg->data;
    forwardGoalReceived = true;
    // mtx.unlock();
}
