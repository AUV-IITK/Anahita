#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Float32.h>
#include <dynamic_reconfigure/server.h>
#include <pid_calibration/pidConfig.h>
#include <pid_calibration/errorToPWM.h>
 
double x;
double y;
double z;
double roll;
double pitch;
double yaw;

bool send_goal = false;
double p = 0;
double i = 0;
double d = 0;
double target = 0;
double band = 0;

ErrorDescriptor yaw_ ("YAW");
ErrorDescriptor roll_ ("ROLL");
ErrorDescriptor pitch_ ("PITCH");
ErrorDescriptor surge ("SURGE");
ErrorDescriptor sway ("SWAY");
ErrorDescriptor heave ("HEAVE");

std::string action = "roll";

void xCB (const std_msgs::Float32 msg) {
    x = msg.data;
    surge.errorToPWM (x);
}

void yCB (const std_msgs::Float32 msg) {
    y = msg.data;
    sway.errorToPWM (y);
}

void zCB (const std_msgs::Float32 msg) {
    z = msg.data;
    heave.errorToPWM (z);
}

void rollCB (const std_msgs::Float32 msg) {
    roll = msg.data;
    roll_.errorToPWM (roll);
}

void pitchCB (const std_msgs::Float32 msg) {
    pitch = msg.data;
    pitch_.errorToPWM (pitch);
}

void yawCB (const std_msgs::Float32 msg) {
    yaw = msg.data;
    yaw_.errorToPWM(yaw);
}

void setTarget (double target) {
    
    if (action == "yaw") { yaw_.setReference (target); }
    if (action == "roll") { roll_.setReference (target); }
    if (action == "pitch") { pitch_.setReference (target); }
    if (action == "surge") { surge.setReference (target); }
    if (action == "heave") { heave.setReference (target); }
    if (action == "sway") { sway.setReference (target); }
}

void callback (pid_calibration::pidConfig &config, uint32_t level) {
    target = config.target;
    if (config.send_goal) {
        setTarget (target);
        ROS_INFO("Got a target request: %f", target);
        config.send_goal = false;
    }
    p = config.p;
    i = config.i;
    d = config.d;
    band = config.band;

    ROS_INFO ("PID reconfigure request, P I D: %f %f %f", p, i, d);

    if (action == "yaw") { yaw_.setPID (p, i, d, band); }
    if (action == "roll") { roll_.setPID (p, i, d, band); }
    if (action == "pitch") { pitch_.setPID (p, i, d, band); }
    if (action == "surge") { surge.setPID (p, i, d, band); }
    if (action == "heave") { sway.setPID (p, i, d, band); }
    if (action == "sway") { heave.setPID (p, i, d, band); }
}

int main (int argc, char** argv) {
    ros::init (argc, argv, "pid_calibrate");
    ros::NodeHandle nh;

    ros::Subscriber x_sub = nh.subscribe<std_msgs::Float32>("/anahita/x_coordinate", 1, &xCB);
    ros::Subscriber z_sub = nh.subscribe<std_msgs::Float32>("/anahita/z_coordinate", 1, &zCB);
    ros::Subscriber y_sub = nh.subscribe<std_msgs::Float32>("/anahita/y_coordinate", 1, &yCB);
    ros::Subscriber roll_sub = nh.subscribe<std_msgs::Float32>("/anahita/imu/roll", 1, &rollCB);
    ros::Subscriber pitch_sub = nh.subscribe<std_msgs::Float32>("/anahita/imu/pitch", 1, &pitchCB);
    ros::Subscriber yaw_sub = nh.subscribe<std_msgs::Float32>("/anahita/imu/yaw", 1, &yawCB); 
    
    // register dynamic reconfig server.
    dynamic_reconfigure::Server<pid_calibration::pidConfig> server;
    dynamic_reconfigure::Server<pid_calibration::pidConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::Rate loop_rate(50);

    int pwm_yaw = 0;
    int pwm_roll = 0;
    int pwm_pitch = 0;
    int pwm_surge = 0;
    int pwm_heave = 0;
    int pwm_sway = 0;

    while (ros::ok()) {

        if (action == "sway") { pwm_sway = sway.getPWM (); }
        if (action == "surge") { pwm_surge = surge.getPWM (); }
        if (action == "heave") { pwm_heave = heave.getPWM (); }
        if (action == "roll") { pwm_roll = roll_.getPWM (); }
        if (action == "pitch") { 
            pwm_pitch = pitch_.getPWM (); 
            std::cout << "pwm_pitch: " << pwm_pitch << std::endl;
        }
        if (action == "yaw") { pwm_yaw = yaw_.getPWM (); }

        nh.setParam ("/pwm_sway", pwm_sway);
        nh.setParam ("/pwm_surge", pwm_surge);
        nh.setParam ("/pwm_heave", pwm_heave);
        nh.setParam ("/pwm_roll", pwm_roll);
        nh.setParam ("/pwm_yaw", pwm_yaw);
        nh.setParam ("/pwm_pitch", pwm_pitch);

        loop_rate.sleep();

        ros::spinOnce();
    }

    return 0;
}