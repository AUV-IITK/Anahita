#include <gate.h>

gateTask::gateTask(): forwardPIDClient("forwardPID"), sidewardPIDClient("sidewardPID"), anglePIDClient("turnPID") {
    forward_sub_ = nh_.subscribe("/gate_task/gate_coordinates", 1, &gateTask::forwardCB, this);
    sideward_sub_ = nh_.subscribe("/gate_task/gate_coordinates", 1, &gateTask::sidewardCB, this);
    angle_sub_ = nh_.subscribe("/varun/sensors/imu/yaw", 1, &gateTask::angleCB, this);   
    gate_status_ = nh_.subscribe("/gate_task/done", 1, &gateTask::statusCB, this); 
    angleGoalReceived = false;
    gateTask_done = false;
}

gateTask::~gateTask() {

}

void gateTask::setActive(bool status) {
    if (status) {
        spin_thread = new boost::thread(boost::bind(&gateTask::spinThread, this));
    }
    else {
        spin_thread->join();
    }
}

void gateTask::spinThread() {
    ROS_INFO("Waiting for sidewardPID server to start.");
    sidewardPIDClient.waitForServer();

    ROS_INFO("sidewardPID server started, sending goal.");
    sidewardPIDgoal.target_distance = 0;
    sidewardPIDClient.sendGoal(sidewardPIDgoal);

    ///////////////////////////////////////////////////

    ROS_INFO("Waiting for anglePID server to start.");
    anglePIDClient.waitForServer();

    ROS_INFO("anglePID server stated");

    /////////////////////////////////////////////////////

    nh_.setParam("/pwm_forward_right", 100);
    nh_.setParam("/pwm_forward_left", 100);

    while(forward_distance_ >= 100) {
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

}

void gateTask::forwardCB(const geometry_msgs::PointStamped::ConstPtr &_msg) {
    forward_distance_ = _msg->point.x;
}

void gateTask::sidewardCB(const geometry_msgs::PointStamped::ConstPtr &_msg) {
    sideward_distance_ = _msg->point.y;
}

void gateTask::angleCB(const std_msgs::Float64Ptr &_msg) {
    angle_ = _msg->data;
    if (angleGoalReceived) {
        ROS_INFO("anglePID server sending goal.");
        anglePIDGoal.target_angle = angle_;
        anglePIDClient.sendGoal(anglePIDGoal);
        angleGoalReceived = false;
    }
}

void gateTask::statusCB(const std_msgs::BoolPtr &_msg) {
    gateTask_done = _msg->data;
}
