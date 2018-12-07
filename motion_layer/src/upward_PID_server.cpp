#include <upward_PID_server.h>

upwardPIDAction::upwardPIDAction(std::string name) :
    as_(nh_, name, false),
    action_name_(name), z_coord("Z_COORD")
{
    //register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&upwardPIDAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&upwardPIDAction::preemptCB, this));
    goal_ = 0;

    //subscribe to the data topic of interest
    sub_ = nh_.subscribe("/anahita/z_coordinate", 1, &upwardPIDAction::depthCB, this);
    pub_ = nh_.advertise<std_msgs::Bool>("/kill/linearvelocity/z", 1);
    z_coord.setPID(7.5, 0, 2, 10);

    as_.start();
    ROS_INFO("upward_PID_server Initiated");
}

upwardPIDAction::~upwardPIDAction(void)
{
}

void upwardPIDAction::goalCB()
{
    goal_ = as_.acceptNewGoal()->target_depth;
    z_coord.setReference(goal_);

    // publish info to the console for the user
    ROS_INFO("%s: Executing, got a target of %f", action_name_.c_str(), goal_);
}

void upwardPIDAction::preemptCB()
{
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
}

void upwardPIDAction::depthCB(const std_msgs::Float32ConstPtr& msg)
{
    // make sure that the action hasn't been canceled
    if (!as_.isActive())
        return;
    
    z_coord.errorToPWM(msg->data);

    feedback_.current_depth = msg->data;
    as_.publishFeedback(feedback_);

    if (msg->data == goal_) {
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
        std_msgs::Bool msg_;
        msg_.data = true;
        pub_.publish(msg_);
    }

    nh_.setParam("/pwm_upward_front", z_coord.getPWM());
    nh_.setParam("/pwm_upward_back", z_coord.getPWM());
}

