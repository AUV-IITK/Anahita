#include <single_buoy.h>

singleBuoy::singleBuoy(): forwardPIDClient("forwardPID"), sidewardPIDClient("sidewardPID"), anglePIDClient("turnPID") {
    forward_sub_ = nh_.subscribe("/buoy_task/buoy_coordinates", 1, &singleBuoy::forwardCB, this);
    sideward_sub_ = nh_.subscribe("/buoy_task/buoy_coordinates", 1, &singleBuoy::sidewardCB, this);
    angle_sub_ = nh_.subscribe("/varun/sensors/imu/yaw", 1, &singleBuoy::angleCB, this);    
    angleGoalReceived = false;
}
singleBuoy::~singleBuoy() {}

void singleBuoy::setActive(bool status) {

    if (status) {
        // spin_thread = new boost::thread(boost::bind(&singleBuoy::spinThread, this));
        ROS_INFO("Waiting for sidewardPID server to start.");
        sidewardPIDClient.waitForServer();

        ROS_INFO("sidewardPID server started, sending goal.");
        sidewardPIDgoal.target_distance = 0;
        sidewardPIDClient.sendGoal(sidewardPIDgoal);

        ///////////////////////////////////////////////////

        ROS_INFO("Waiting for anglePID server to start.");
        anglePIDClient.waitForServer();

        while (!angleGoalReceived) {}

        ROS_INFO("anglePID server started, sending goal.");

        anglePIDGoal.target_angle = angle_;
        anglePIDClient.sendGoal(anglePIDGoal);

        /////////////////////////////////////////////////////

        nh_.setParam("/pwm_forward_right", 100);
        nh_.setParam("/pwm_forward_left", 100);

        while(forward_distance_ >= 60) {
            continue;
        }
        sidewardPIDClient.cancelGoal();
        ros::Duration(6).sleep();

        nh_.setParam("/pwm_forward_right", -100);
        nh_.setParam("/pwm_forward_left", -100);

        ros::Duration(10).sleep();

        //////////////////////////////////////////////////////

        ROS_INFO("Waiting for action server to start.");
        forwardPIDClient.waitForServer();

        ROS_INFO("Action server started, sending goal.");
        
        forwardPIDgoal.target_distance = 100;
        forwardPIDClient.sendGoal(forwardPIDgoal);
        while(forward_distance_ <= 100) {
            continue;
        }
        forwardPIDClient.cancelGoal();

        nh_.setParam("/pwm_forward_right", 0);
        nh_.setParam("/pwm_forward_left", 0);
        nh_.setParam("/pwm_sideward_front", 0);
        nh_.setParam("/pwm_sideward_back", 0);

        anglePIDClient.cancelGoal();
    }
    else {
        // spin_thread->join();
    }
}

// void singleBuoy::spinThread() {
// }

void singleBuoy::forwardCB(const geometry_msgs::PointStamped::ConstPtr &_msg) {
    forward_distance_ = _msg->point.x;
}

void singleBuoy::sidewardCB(const geometry_msgs::PointStamped::ConstPtr &_msg) {
    sideward_distance_ = _msg->point.y;
}

void singleBuoy::angleCB(const std_msgs::Float64Ptr &_msg) {
    angle_ = _msg->data;
    angleGoalReceived = true;
}
