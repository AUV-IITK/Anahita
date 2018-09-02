#include <sideward_PID_server.h>

sidewardPIDAction::sidewardPIDAction(std::string name) :
    as_(nh_, name, false),
    action_name_(name), y_coord("Y_COORD")
{
    //register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&sidewardPIDAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&sidewardPIDAction::preemptCB, this));
    goal_ = 0;

    y_coord.setPID(7.5, 0, 2, 10);
    
    //subscribe to the data topic of interest
    sub_ = nh_.subscribe("/buoy_task/buoy_coordinates", 1, &sidewardPIDAction::visionCB, this);

    as_.start();
}

sidewardPIDAction::~sidewardPIDAction(void)
{
}

void sidewardPIDAction::goalCB()
{
    goal_ = as_.acceptNewGoal()->target_distance;
    y_coord.setReference(goal_);

    // publish info to the console for the user
    ROS_INFO("%s: Executing, got a target of %f", action_name_.c_str(), goal_);
}

void sidewardPIDAction::preemptCB()
{
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
}

void sidewardPIDAction::visionCB(const geometry_msgs::PointStampedConstPtr &msg) {
    if (!as_.isActive())
        return;
    
    y_coord.errorToPWM(msg->point.y);

    feedback_.current_distance = msg->point.y;
    as_.publishFeedback(feedback_);

    if (msg->point.y == goal_) {
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
    }

    nh_.setParam("/pwm_sideward_front_straight", y_coord.getPWM());
    nh_.setParam("/pwm_sideward_back_straight", y_coord.getPWM());
}

