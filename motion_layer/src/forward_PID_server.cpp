#include <forward_PID_server.h>

forwardPIDAction::forwardPIDAction(std::string name) :
    as_(nh_, name, false),
    action_name_(name), x_coord("X_COORD")
{
    //register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&forwardPIDAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&forwardPIDAction::preemptCB, this));
    goal_ = 0;

    x_coord.setPID(3.75, 0.0, 0.0, 5);
    
    //subscribe to the data topic of interest
    sub_ = nh_.subscribe("/anahita/x_coordinate", 1, &forwardPIDAction::visionCB, this);

    as_.start();
    ROS_INFO("forward_PID_server Initialised");
}

forwardPIDAction::~forwardPIDAction(void)
{
}

void forwardPIDAction::goalCB()
{
    goal_ = as_.acceptNewGoal()->target_distance;

    x_coord.setReference(goal_);

    // publish info to the console for the user
    ROS_INFO("%s: Executing, got a target of %f", action_name_.c_str(), goal_);
}

void forwardPIDAction::preemptCB()
{
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
}

void forwardPIDAction::visionCB(const std_msgs::Float32ConstPtr &msg) {
    if (!as_.isActive())
        return;
    
    x_coord.errorToPWM(msg->data);

    feedback_.current_distance = msg->data;

    as_.publishFeedback(feedback_);

    // if (msg->data <= goal_ + 15 && msg->data >= goal_ - 15) {
        // ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        // as_.setSucceeded(result_);
    // }

    nh_.setParam("/pwm_surge", x_coord.getPWM());
}
