#include <surge_PID_server.h>

surgePIDAction::surgePIDAction(std::string name) :
    as_(nh_, name, false),
    action_name_(name), surge("SURGE")
{
    // register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&surgePIDAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&surgePIDAction::preemptCB, this));
    goal_ = 0;

    double p, i, d, band;

    nh_.getParam("/surge/p", p);
    nh_.getParam("/surge/i", i);
    nh_.getParam("/surge/d", d);
    nh_.getParam("/surge/band", band);

    surge.setPID(p, i, d, band);
    
    // subscribe to the data topic of interest
    sub_ = nh_.subscribe("/anahita/x_coordinate", 1, &surgePIDAction::callback, this);

    as_.start();
    ROS_INFO("surge_PID_server Initialised");
}

surgePIDAction::~surgePIDAction(void)
{
}

void surgePIDAction::goalCB()
{
    goal_ = as_.acceptNewGoal()->target_surge;

    surge.setReference(goal_);

    // publish info to the console for the user
    ROS_INFO("%s: Executing, got a target of %f", action_name_.c_str(), goal_);
}

void surgePIDAction::preemptCB()
{
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
}

void surgePIDAction::callback(const std_msgs::Float32ConstPtr &msg) {
    if (!as_.isActive())
        return;
    
    surge.errorToPWM(msg->data);

    feedback_.current_surge = msg->data;

    as_.publishFeedback(feedback_);

    nh_.setParam("/pwm_surge", surge.getPWM());
}
