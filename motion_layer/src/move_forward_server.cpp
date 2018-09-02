#include <move_forward_server.h>

moveForward::moveForward(int pwm_): upwardPIDClient_sensor_("upwardPID/sensor"), anglePIDClient_sensor_("turnPID/sensor"),
                                    sidewardPIDClient_("sidewardPID"), upwardPIDClient_vision_("upwardPID/vision"), 
                                    anglePIDClient_vision_("turnPID/vision")
{
    spin_thread = new boost::thread(boost::bind(&moveForward::spinThread, this));
    angle_sub_ = nh.subscribe("/varun/sensors/imu/yaw", 1, &moveForward::imuAngleCB, this);
    depth_sub_ = nh.subscribe("/varun/sensors/depth", 1, &moveForward::depthCB, this);
    nh.setParam("/pwm_forward_right", pwm_);
    nh.setParam("/pwm_forward_left", pwm_);
}

moveForward::~moveForward() {
    spin_thread->join();
}

void moveForward::setActive(bool status) {
    if (status == true) {
        ROS_INFO("Waiting for sidewardPID server to start.");
        sidewardPIDClient_.waitForServer();

        ROS_INFO("sidewardPID server started, sending goal.");
        sideward_PID_goal.target_distance = 0;
        sidewardPIDClient_.sendGoal(sideward_PID_goal);

        if (upward_type_ == "VISION") {
            ROS_INFO("Waiting for upwardPID/vision server to start.");
            upwardPIDClient_vision_.waitForServer();

            ROS_INFO("upwardPID/vision server started, sending goal.");
            upward_PID_goal.target_depth = depth;
            upwardPIDClient_vision_.sendGoal(upward_PID_goal);
        }
        else if (upward_type_ == "SENSOR") {
            ROS_INFO("Waiting for upwardPID server to start.");
            upwardPIDClient_sensor_.waitForServer();

            ROS_INFO("upwardPID server started, sending goal.");
            upward_PID_goal.target_depth = depth;
            upwardPIDClient_sensor_.sendGoal(upward_PID_goal);
        }

        if (angle_type_ == "VISION") {
            ROS_INFO("Waiting for anglePID server to start.");
            anglePIDClient_vision_.waitForServer();

            ROS_INFO("anglePID server started, sending goal.");
            angle_PID_goal.target_angle = 0;
            anglePIDClient_vision_.sendGoal(angle_PID_goal);
        }
        else if (angle_type_ == "SENSOR") {
            ROS_INFO("Waiting for anglePID server to start.");
            anglePIDClient_sensor_.waitForServer();

            ROS_INFO("anglePID server started, sending goal.");
            angle_PID_goal.target_angle = angle;
            anglePIDClient_sensor_.sendGoal(angle_PID_goal);
        }

    }

    if (status == false) {
        sidewardPIDClient_.cancelGoal();
        upwardPIDClient_sensor_.cancelGoal();
        upwardPIDClient_vision_.cancelGoal();
        anglePIDClient_sensor_.cancelGoal();
        anglePIDClient_vision_.cancelGoal();
    }
}   

void moveForward::spinThread() {
    ros::spin();
}

void moveForward::setReferenceAngle(double angle_) {
    angle = angle_;
}

void moveForward::setReferenceDepth(double depth_) {
    depth = depth_;
}

void moveForward::setDataSource(std::string angle_type, std::string upward_type) {
    angle_type_ = angle_type;
    upward_type_ = upward_type;
}

void moveForward::imuAngleCB(const std_msgs::Float64Ptr &_msg) {
    angle = _msg->data;
}

void moveForward::depthCB(const std_msgs::Float64Ptr &_msg) {
    depth = _msg->data;
}