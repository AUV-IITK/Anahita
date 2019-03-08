#include <sway_PID_server.h>

swayPIDAction::swayPIDAction(std::string name) :
    as_(nh_, name, false),
    action_name_(name), sway("SWAY")
{
    // register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&swayPIDAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&swayPIDAction::preemptCB, this));
    goal_ = 0;
    
    // subscribe to the data topic of interest
    sub_ = nh_.subscribe("/anahita/y_coordinate", 1, &swayPIDAction::callback, this);

    double p, i, d, band;

    nh_.getParam("/sway/p", p);
    nh_.getParam("/sway/i", i);
    nh_.getParam("/sway/d", d);
    nh_.getParam("/sway/band", band);

    sway.setPID(p, i, d, band);

    as_.start();
    ROS_INFO("sway_PID_server Initiated");
}

swayPIDAction::~swayPIDAction(void)
{
}

void swayPIDAction::goalCB()
{
    goal_ = as_.acceptNewGoal()->target_sway;
    sway.setReference(goal_);

    // publish info to the console for the user
    ROS_INFO("%s: Executing, got a target of %f", action_name_.c_str(), goal_);
}

void swayPIDAction::preemptCB()
{
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
}

void swayPIDAction::callback(const std_msgs::Float32ConstPtr &msg) {

    if (!as_.isActive())
        return;
    
    sway.errorToPWM(msg->data);

    feedback_.current_sway = msg->data;
    as_.publishFeedback(feedback_);

    nh_.setParam("/pwm_sway", sway.getPWM());
}

