#include <angle_PID_server.h>

anglePIDAction::anglePIDAction(std::string name) :
    as_(nh_, name, false),
    action_name_(name), angle("ANGLE")
{
    // register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&anglePIDAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&anglePIDAction::preemptCB, this));
    goal_ = 0;

    std::cout << "calliing 1" << std::endl;

    // type = type_;
    
    // subscribe to the data topic of interest
    sub_ = nh_.subscribe("/varun/sensors/imu/yaw", 1, &anglePIDAction::callBack, this);
    angle.setPID(2.4, 0, 0.5, 1);

    as_.start();
}

anglePIDAction::~anglePIDAction(void)
{
}

void anglePIDAction::goalCB()
{
    goal_ = as_.acceptNewGoal()->target_angle;

    angle.setReference(goal_);

    // publish info to the console for the user
    ROS_INFO("%s: Executing, got a target of %f", action_name_.c_str(), goal_);
}

void anglePIDAction::preemptCB()
{
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
}

void anglePIDAction::callBack(const std_msgs::Float32::ConstPtr& msg)
{
    // make sure that the action hasn't been canceled
    if (!as_.isActive()) {
        std::cout << "hfksdh" << std::endl;
        return;
    }
    
    ROS_INFO("Calling");
    angle.errorToPWM(msg->data);

    feedback_.current_angle = msg->data;
    as_.publishFeedback(feedback_);

    if (msg->data == goal_) {
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
    }

    nh_.setParam("/pwm_sideward_front_turn", angle.getPWM());
    nh_.setParam("/pwm_sideward_back_turn", -1*angle.getPWM());
}

// void anglePIDAction::visionCB(const geometry_msgs::Pose2DConstPtr &msg) {
//     if (!as_.isActive())
//         return;
    
//     angle.errorToPWM(msg->theta);

//     feedback_.current_angle = msg->theta;
//     as_.publishFeedback(feedback_);

//     if (msg->theta == goal_) {
//         ROS_INFO("%s: Succeeded", action_name_.c_str());
//         // set the action state to succeeded
//         as_.setSucceeded(result_);
//     }

//     nh_.setParam("/pwm_sideward_front_turn", angle.getPWM());
//     nh_.setParam("/pwm_sideward_back_turn", -1*angle.getPWM());
// }

void anglePIDAction::setDataSource(std::string type_) {
    type = type_;
}
