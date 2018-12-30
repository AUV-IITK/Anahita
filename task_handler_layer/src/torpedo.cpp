#include <torpedo.h>

Torpedo::Torpedo(): forwardPIDClient("forwardPID"), sidewardPIDClient("sidewardPID"), 
                        anglePIDClient("turnPID"), upwardPIDClient("upwardPID"), th(15)
{
    forward_sub_ = nh_.subscribe("/anahita/x_coordinate", 1, &Torpedo::forwardCB, this);
    sideward_sub_ = nh_.subscribe("/anahita/y_coordinate", 1, &Torpedo::sidewardCB, this);
    upward_sub_ = nh_.subscribe("/anahita/z_coordinate", 1, &Torpedo::upwardCB, this);
    angle_sub_ = nh_.subscribe("/mavros/imu/yaw", 1, &Torpedo::angleCB, this);
    // spin_thread = new boost::thread(boost::bind(&Torpedo::spinThread, this));
}

Torpedo::~Torpedo() {}

void Torpedo::setActive(bool status) {

    if (status) {

        ROS_INFO("Waiting for sidewardPID server to start, task torpedo.");
        sidewardPIDClient.waitForServer();

        ROS_INFO("sidewardPID server started, sending goal, task torpedo.");
        sidewardPIDgoal.target_distance = 0;
        sidewardPIDClient.sendGoal(sidewardPIDgoal);

        ///////////////////////////////////////////////////

        ROS_INFO("Waiting for upwardPID server to start, task torpedo.");
        upwardPIDClient.waitForServer();

        ROS_INFO("upwardPID server started, sending goal, task torpedo.");
        upwardPIDgoal.target_depth = 0; // for gazebo
        upwardPIDClient.sendGoal(upwardPIDgoal);

        ///////////////////////////////////////////////////

        ROS_INFO("Waiting for anglePID server to start, task torpedo.");
        anglePIDClient.waitForServer();

        while (!angleGoalReceived) {}

        ROS_INFO("anglePID server started, sending goal, task torpedo.");

        anglePIDGoal.target_angle = 0;
        anglePIDClient.sendGoal(anglePIDGoal);

        /////////////////////////////////////////////////////

        while (!forwardGoalReceived && ros::ok()) {}

        ROS_INFO("forward distance received");

        ROS_INFO("ForwardPID Client sending goal, task torpedo.");        
        forwardPIDgoal.target_distance = 50;
        forwardPIDClient.sendGoal(forwardPIDgoal);

        th.isAchieved(50, 15, "forward");

        ROS_INFO("Killing the thrusters");
	    nh_.setParam("/kill_signal", true);
        nh_.setParam("/kill_signal", false);

        forwardPIDClient.cancelGoal();

        // fire the torpedo

        ROS_INFO("ForwardPID Client sending goal again, task torpedo.");        
        forwardPIDgoal.target_distance = 100;
        forwardPIDClient.sendGoal(forwardPIDgoal);
    }
    else {
        // spin_thread->join();
        forwardPIDClient.cancelGoal();
        upwardPIDClient.cancelGoal();
        sidewardPIDClient.cancelGoal();
        anglePIDClient.cancelGoal();
        ROS_INFO("Closing Single Buoy");
    }
}

// void Torpedo::spinThread() {
//     ros::spin();
// }

void Torpedo::forwardCB(const std_msgs::Float32ConstPtr &_msg) {
    forward_distance_ = _msg->data;
    forwardGoalReceived = true;
}

void Torpedo::sidewardCB(const std_msgs::Float32ConstPtr &_msg) {
    sideward_distance_ = _msg->data;
}

void Torpedo::upwardCB(const std_msgs::Float32Ptr &_msg) {
    depth_ = _msg->data;
}

void Torpedo::angleCB(const std_msgs::Float32Ptr &_msg) {
    angle_ = _msg->data;
    angleGoalReceived = true;
}
