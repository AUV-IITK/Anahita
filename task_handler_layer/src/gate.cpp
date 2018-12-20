#include <gate.h>

gateTask::gateTask(): forwardPIDClient("forwardPID"), sidewardPIDClient("sidewardPID"), anglePIDClient("turnPID") {
    forward_sub_ = nh_.subscribe("/gate_task/gate_coordinates", 1, &gateTask::forwardCB, this);
    sideward_sub_ = nh_.subscribe("/gate_task/gate_coordinates", 1, &gateTask::sidewardCB, this);
    angle_sub_ = nh_.subscribe("/varun/sensors/imu/yaw", 1, &gateTask::angleCB, this);
    angleGoalReceived = false;
    spin_thread = new boost::thread(boost::bind(&gateTask::spinThread, this));
}

gateTask::~gateTask() {

}

void gateTask::setActive(bool status) {
    if (status) {
        ROS_INFO("Waiting for sidewardPID server to start, Gate Task.");
        sidewardPIDClient.waitForServer();

        ROS_INFO("sidewardPID server started, sending goal, Gate Task.");
        sidewardPIDgoal.target_distance = 0;
        sidewardPIDClient.sendGoal(sidewardPIDgoal);

        ///////////////////////////////////////////////////

        ROS_INFO("Waiting for anglePID server to start.");
        anglePIDClient.waitForServer();

        ROS_INFO("anglePID server stated");

        while (!angleGoalReceived) {}

        ROS_INFO("anglePID server started, sending goal.");

        anglePIDGoal.target_angle = angle_;
        anglePIDClient.sendGoal(anglePIDGoal);

        /////////////////////////////////////////////////////

        nh_.setParam("/pwm_surge", 100);
        nh_.setParam("/pwm_surge", 100);

        while(forward_distance_ >= 100) {
            continue;
        }

        sidewardPIDClient.cancelGoal();
        ros::Duration(6).sleep();

        anglePIDClient.cancelGoal();

        nh_.setParam("/kill_signal", true);
    }
    else {
        spin_thread->join();
    }
}

void gateTask::spinThread() {
    ros::spin();
}

void gateTask::forwardCB(const std_msgs::Float32Ptr &_msg) {
    forward_distance_ = _msg->data;
}

void gateTask::sidewardCB(const std_msgs::Float32Ptr &_msg) {
    sideward_distance_ = _msg->data;
}

void gateTask::angleCB(const std_msgs::Float32Ptr &_msg) {
    angle_ = _msg->data;
    angleGoalReceived = true;
}
