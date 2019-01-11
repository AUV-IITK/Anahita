#include <navigation_handler.h>

navigationHandler::navigationHandler () : move_straight(0), move_sideward(0), move_downward(0), th(15),
                                        anglePIDClient("turnPID"), upwardPIDClient("upwardPID"), 
                                        forwardPIDClient("forwardPID"), sidewardPIDClient("sidewardPID") {}

navigationHandler::~navigationHandler () {}

void navigationHandler::stabilise (std::string type) {
    
    double depth = 0;
    nh.getParam("/reference_depth", depth);
    nh.setParam("/enable_pressure", true);

    ROS_INFO("Waiting for upwardPID server to start, Navigation Handle");
    upwardPIDClient.waitForServer();

    ROS_INFO("upwardPID server started, sending goal.");
    upwardPIDGoal.target_depth = depth;
    upwardPIDClient.sendGoal(upwardPIDGoal);

    if (!th.isAchieved(depth, 10, "upward")) {
        ROS_INFO("Time limit exceeded for upward");
        return;
    }
    
    move_straight.setThrust(0);
    move_straight.setActive(true, type);
    ros::Duration(7).sleep();
    move_straight.setActive(false, type);

    nh.setParam("/enable_pressure", false);
    upwardPIDClient.cancelGoal();
}

void navigationHandler::manouver () {
    nh.setParam("/set_local_yaw", true);
    nh.setParam("/use_local_yaw", true);

    ROS_INFO("Waiting for anglePID server to start, Navigation Handle");
    anglePIDClient.waitForServer();

    ROS_INFO("anglePID server started, sending goal");
    anglePIDGoal.target_angle = 0;
    anglePIDClient.sendGoal(anglePIDGoal);

    if (!th.isAchieved(0, 2, "angle")) {
        ROS_INFO("Time limit exceeded for angle, Navigation Handle");
        return;
    }

    // double depth = 0;
    // nh.getParam("/reference_depth", depth);
    // nh.setParam("/enable_pressure", true);

    // ROS_INFO("Waiting for upwardPID server to start, Navigation Handle");
    // upwardPIDClient.waitForServer();

    // ROS_INFO("upwardPID server started, sending goal");
    // upwardPIDGoal.target_depth = depth;
    // upwardPIDClient.sendGoal(upwardPIDGoal); 

    // if (!th.isAchieved(depth, 10, "upward")) {
    //     ROS_INFO("Time limit exceeded for upward, Navigation Handle");
    //     return;
    // }

    move_straight.setThrust(50);
    move_straight.setActive(true, "local_yaw");

    nh.setParam("/pwm_sway", 50);

    ros::Duration(6).sleep();

    move_straight.setActive(false, "local_yaw");
    nh.setParam("/pwm_sway", 0);

    move_straight.setThrust(50);
    move_straight.setActive(true, "local_yaw");

    nh.setParam("/pwm_sway", -50);

    ros::Duration(4).sleep();

    move_straight.setActive(false, "local_yaw");
    nh.setParam("/pwm_sway", 0);

    ros::Duration(3).sleep();

    move_straight.setThrust(-50);
    move_straight.setActive(true, "local_yaw");

    ros::Duration(10).sleep();

    move_straight.setActive(false, "local_yaw");

    anglePIDClient.cancelGoal();
    // upwardPIDClient.cancelGoal();

    // nh.setParam("/enable_pressure", false);
    nh.setParam("/use_local_yaw", false);
}

bool navigationHandler::find (std::string object) {
    spin_thread = new boost::thread(boost::bind(&navigationHandler::manouver, this));
    nh.setParam("/current_task", object);
    if (!th.isDetected(object, 30)) {
        ROS_INFO("Not found");
        return false;
    }
    ROS_INFO("object detected");
    spin_thread-join();
    return true;
}

bool navigationHandler::isStable () {
    ros::Rate loop_rate(50);

    double time_out = 6; // time out 

    double then = ros::Time::now().toSec();
    double now = ros::Time::now().toSec();
    double start = ros::Time::now().toSec();

    double diff = 0;
    double time = 0;

    int count = 0;

    int _band = 75; // diff in subsequent pwms

    int pwm_sway_prev, pwm_surge_prev, pwm_heave_prev, pwm_yaw_prev, pwm_roll_prev, pwm_pitch_prev;
    int pwm_sway_curr, pwm_surge_curr, pwm_heave_curr, pwm_yaw_curr, pwm_roll_curr, pwm_pitch_curr;
    int pwm_sway_diff, pwm_surge_diff, pwm_heave_diff, pwm_yaw_diff, pwm_roll_diff, pwm_pitch_diff;

    while (ros::ok()) {

        now = ros::Time::now().toSec();
        diff = now - then;
        time = now - start;

        nh.getParam("/pwm_surge", pwm_surge_curr);
        nh.getParam("/pwm_sway", pwm_sway_curr);
        nh.getParam("/pwm_heave", pwm_heave_curr);
        nh.getParam("/pwm_yaw", pwm_yaw_curr);
        nh.getParam("/pwm_roll", pwm_roll_curr);
        nh.getParam("/pwm_pitch", pwm_pitch_curr);

        pwm_surge_diff =  pwm_surge_prev - pwm_surge_curr;
        pwm_sway_diff = pwm_sway_prev - pwm_sway_curr;
        pwm_heave_diff = pwm_heave_prev - pwm_heave_curr;
        pwm_yaw_diff = pwm_yaw_prev - pwm_yaw_curr;
        pwm_roll_diff = pwm_roll_prev - pwm_roll_curr;
        pwm_pitch_diff = pwm_pitch_prev - pwm_pitch_curr;

        if (pwm_surge_diff >= _band || pwm_surge_curr > 200) {
            if (!count) {
                then = ros::Time::now().toSec();
            }
            count++;
        }
        if (pwm_sway_diff >= _band || pwm_sway_curr > 200) {
            if (!count) {
                then = ros::Time::now().toSec();
            }
            count++;
        }
        if (pwm_heave_diff >= _band || pwm_heave_curr > 200) {
            if (!count) {
                then = ros::Time::now().toSec();
            }
            count++;
        }
        if (pwm_roll_diff >= _band || pwm_roll_curr > 200) {
            if (!count) {
                then = ros::Time::now().toSec();
            }
            count++;
        }
        if (pwm_pitch_diff >= _band || pwm_pitch_curr > 200) {
            if (!count) {
                then = ros::Time::now().toSec();
            }
            count++;
        }
        if (pwm_yaw_diff >= _band || pwm_yaw_curr > 200) {
            if (!count) {
                then = ros::Time::now().toSec();
            }
            count++;
        }

        if (diff >= 2.0) {
            if (count >= 50) {
                return false;
            }
            else {
                count = 0;
            }
        }

        if (time >= time_out) {
            return true;
        }

        pwm_surge_prev = pwm_surge_curr;
        pwm_sway_prev = pwm_sway_curr;
        pwm_heave_prev = pwm_heave_curr;
        pwm_yaw_prev = pwm_yaw_curr;
        pwm_roll_prev = pwm_roll_curr;
        pwm_pitch_prev = pwm_pitch_curr;
        
        loop_rate.sleep();
    }
}

void navigationHandler::findNextTask () {}

void navigationHandler::analyze () {}
