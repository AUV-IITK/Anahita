#include <navigation_handler.h>

navigationHandler::navigationHandler () : th(20), yawPIDClient("yawPID"), heavePIDClient("heavePID"), 
                                        surgePIDClient("surgePID"), swayPIDClient("swayPID") {}

navigationHandler::~navigationHandler () {}

void navigationHandler::stabilise (std::string type) {
    
    depth_stabilise.activate("reference");
    move_straight.activate(0, type);
    
    ros::Duration(7).sleep();

    if (!th.isAchieved(0, 10, "heave")) {
        ROS_INFO("Time limit exceeded for heave");
        return;
    }

    move_straight.deActivate ();
    depth_stabilise.deActivate ();
}

void navigationHandler::manouver () {

    nh.setParam("/use_local_yaw", true);

    depth_stabilise.activate("reference");
    nh.setParam("/enable_pressure", true);
    nh.setParam("/disable_imu", false);

    ROS_INFO("Waiting for yawPID server to start, Navigation Handle");
    yawPIDClient.waitForServer();

    ROS_INFO("yawPID server started, sending goal");
    yawPIDGoal.target_yaw = 0;
    yawPIDClient.sendGoal(yawPIDGoal);

    if (!th.isAchieved(0, 2, "yaw")) {
        ROS_INFO("Time limit exceeded for yaw, NV");
        ros::Duration(3).sleep();
        // return;
    }
    yawPIDClient.cancelGoal();

    if (!th.isAchieved(0, 10, "heave")) {
        ROS_INFO("Time limit exceeded for heave, NV");
        // return;
    }

    if (direction == 1) {
        ROS_INFO("Else");
        move_straight.setThrust(50); 
    }
    else { move_straight.setThrust(-50); }

    move_straight.activate(-50, "local_yaw");

    double then = ros::Time::now().toSec();
    double now = ros::Time::now().toSec();
    double diff = 0;

    ros::Rate loop_rate(50);

    bool temp = false;

    if (direction == 1) { ROS_INFO("Moving forward, NV"); }
    else { ROS_INFO("Moving backward, NV"); }

    while (ros::ok()) {
        now = ros::Time::now().toSec();
        diff = now - then;
        if (diff > 5) {
            break;
        }
        mtx.lock();
        temp = stop_manouver;
        mtx.unlock();
        if (temp) {
            move_straight.deActivate ();
            nh.setParam("/kill_signal", true);
            nh.setParam("/use_local_yaw", false);
    	    ROS_INFO("Stopping signal detected");
            return;
        }
        loop_rate.sleep();    
    }
    move_straight.deActivate ();

    then = ros::Time::now().toSec();
    now = ros::Time::now().toSec();
    diff = 0;

    ros::Duration(1).sleep();
    nh.setParam("/pwm_sway", 50);

    ROS_INFO("Moving Right, NV");

    if (direction == 1) { 
        // move_straight.setThrust(50); 
        move_straight.activate(50, "local_yaw");
    }
    else { 
        // move_straight.setThrust(-50); 
        move_straight.activate(-50, "local_yaw");
    }

    while (ros::ok()) {

        now = ros::Time::now().toSec();
        diff = now - then;

        if (diff > 8) {
            break;
        }
        mtx.lock();
        temp = stop_manouver;
        mtx.unlock();
        if (temp) {
            move_straight.deActivate ();
            nh.setParam("/kill_signal", true);
            nh.setParam("/use_local_yaw", false);
	        ROS_INFO("Stopping signal detected");
            return;
        }
        loop_rate.sleep();
    }
    move_straight.deActivate ();
    nh.setParam("/pwm_sway", 0);

    if (direction == 1) { 
        // move_straight.setThrust(35);
        move_straight.activate(35, "local_yaw"); 
    }
    else { 
        // move_straight.setThrust(-35);
        move_straight.activate(-35, "local_yaw");
    }

    nh.setParam("/pwm_sway", -50);
 
    then = ros::Time::now().toSec();
    now = ros::Time::now().toSec();
    diff = 0;

    ROS_INFO("Moving Left, NV");

    while (ros::ok()) {

        now = ros::Time::now().toSec();
        diff = now - then;

        if (diff > 14) {
            break;
        }
        mtx.lock();
        temp = stop_manouver;
        mtx.unlock();
        if (temp) {
            move_straight.deActivate ();
            nh.setParam("/kill_signal", true);
            nh.setParam("/use_local_yaw", false);
	    ROS_INFO("Stopping signal detected");
            return;
        }
        loop_rate.sleep();
    }

    move_straight.deActivate ();
    nh.setParam("/pwm_sway", 0);

    ros::Duration(2).sleep();

    move_straight.setThrust(0);
    move_straight.activate(0, "local_yaw");

    then = ros::Time::now().toSec();
    now = ros::Time::now().toSec();
    diff = 0;

    nh.setParam("/pwm_sway", 50);
    ROS_INFO("Moving only right, NV");

    while (ros::ok()) {

        now = ros::Time::now().toSec();
        diff = now - then;

        if (diff > 7.5) {
            break;
        }

        mtx.lock();
        temp = stop_manouver;
        mtx.unlock();

        if (temp) {
            move_straight.deActivate ();
            nh.setParam("/kill_signal", true);
            nh.setParam("/use_local_yaw", false);
    	    ROS_INFO("Stopping signal detected");
            return;
        }
        loop_rate.sleep();
    }
    move_straight.deActivate ();
    
    if (direction == 1) { 
        // move_straight.setThrust(-50); 
        move_straight.activate (-50, "local_yaw");    
    }
    else { 
        // move_straight.setThrust(50); 
        move_straight.activate (50, "local_yaw");
    }

    then = ros::Time::now().toSec();
    now = ros::Time::now().toSec();
    diff = 0;

    while (ros::ok()) {

        now = ros::Time::now().toSec();
        diff = now - then;

        if (diff > 15) {
            break;
        }

        mtx.lock();
        temp = stop_manouver;
        mtx.unlock();

        if (temp) {
            move_straight.deActivate ();
            nh.setParam("/kill_signal", true);
            nh.setParam("/use_local_yaw", false);
	        ROS_INFO("Stopping signal detected");
            return;
        }
        loop_rate.sleep();
    }

    move_straight.deActivate ();
    depth_stabilise.deActivate ();
    nh.setParam("/use_local_yaw", false);
}

bool navigationHandler::find (std::string object, double time_out, int _direction) {
    direction = _direction;
    spin_thread = new boost::thread(boost::bind(&navigationHandler::manouver, this));
    nh.setParam("/current_task", object);
    std::cout << "NV, Finding Object: " << object << std::endl;
    ros::Duration(1).sleep();
    if (!th.isDetected(object, time_out)) {
        ROS_INFO("Not found");
        mtx.lock();
        stop_manouver = true;
        mtx.unlock();
        return false;
    }
    mtx.lock();
    stop_manouver = true;
    mtx.unlock();
    ROS_INFO("object detected");
    spin_thread->join();
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

bool navigationHandler::scan (std::string object) {
    nh.setParam("/use_reference_yaw", true);
    nh.setParam("/current_task", object);

    depth_stabilise.activate ("reference");

    nh.setParam("/pwm_yaw", 25);
    
    if (th.isDetected(object, 20)) {
        yawPIDClient.cancelGoal();
        nh.setParam("/kill_signal", true);
        return true;
    }

    nh.setParam("/kill_signal", true);
    depth_stabilise.deActivate ();

    return false;
}

bool navigationHandler::dive (std::string target) {

    nh.setParam("/enable_pressure", true);
    nh.setParam("/use_referene_yaw", true);

    ROS_INFO("Waiting for heavePID server to start.");
    heavePIDClient.waitForServer();

    ROS_INFO("heavePID server started, sending goal.");
    heavePIDGoal.target_heave = 40;
    heavePIDClient.sendGoal(heavePIDGoal);

    if (scan(target)) { 
        nh.setParam("/use_reference_yaw", false);
        nh.setParam("/enable_pressure", false);    
        return true; 
    }
    else { 
        nh.setParam("/use_reference_yaw", false);
        nh.setParam("/enable_pressure", false);        
        return false; 
    }
}
