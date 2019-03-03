#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Wrench.h>

int main (int argc, char** argv) {

    ros::init (argc, argv, "gazebo_pwm_pub");
    ros::Time::init();
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::Wrench>("/anahita/thruster_manager/input", 1);

    ros::Rate loop_rate(20);
    geometry_msgs::Wrench wrench;

    int pwm_sway = 0;
    int pwm_surge = 0;
    int pwm_yaw = 0;
    int pwm_roll = 0;
    int pwm_pitch = 0;
    int pwm_heave = 0;

    bool kill_signal = false;

    nh.setParam("/pwm_sway", 0);
    nh.setParam("/pwm_surge", 0);
    nh.setParam("/pwm_heave", 0);
    nh.setParam("/pwm_yaw", 0);
    nh.setParam("/pwm_roll", 0);
    nh.setParam("/pwm_pitch", 0);

    nh.setParam("/kill_signal", false);

    double thrust_0 = 0;
    double thrust_1 = 0;
    double thrust_2 = 0;
    double thrust_3 = 0;
    double thrust_4 = 0; // south-west
    double thrust_5 = 0; // north-west
    double thrust_6 = 0; // north-east
    double thrust_7 = 0; // south-east

    double force_x = 0;
    double force_y = 0;
    double force_z = 0;

    double torque_x = 0;
    double torque_y = 0;
    double torque_z = 0;

    while (ros::ok()) {

        nh.getParam("/pwm_surge", pwm_surge);
        nh.getParam("/pwm_heave", pwm_heave);
        nh.getParam("/pwm_sway", pwm_sway);

        nh.getParam("/pwm_yaw", pwm_yaw);
        nh.getParam("/pwm_roll", pwm_roll);
        nh.getParam("/pwm_pitch", pwm_pitch);

    	nh.getParam("/kill_signal", kill_signal);

        if (kill_signal) {

            ROS_INFO("KILL SIGNAL RECIEVED");
            
            nh.setParam("/pwm_sway", 0);
            nh.setParam("/pwm_surge", 0);
            nh.setParam("/pwm_heave", 0);
            nh.setParam("/pwm_yaw", 0);
            nh.setParam("/pwm_roll", 0);
            nh.setParam("/pwm_pitch", 0);

            nh.setParam("/kill_signal", false);
        }

        thrust_1 = pwm_surge + pwm_yaw;
        thrust_0 = pwm_surge - pwm_yaw;

        thrust_3 = -pwm_sway - pwm_yaw;
        thrust_2 = -pwm_sway + pwm_yaw;

        thrust_6 = -pwm_heave - pwm_roll + pwm_pitch;
        thrust_5 = -pwm_heave + pwm_roll + pwm_pitch;

        thrust_7 = -pwm_heave - pwm_roll - pwm_pitch;
        thrust_4 = -pwm_heave + pwm_roll - pwm_pitch;

        force_x = thrust_0 + thrust_1;
        force_y = thrust_2 + thrust_3;
        force_z = thrust_4 + thrust_5 + thrust_6 + thrust_7;

        torque_x = -0.2036589999986261*thrust_4 + -0.20365899999862605*thrust_5 + 0.20365899999862605*thrust_6 + 0.20365899999862605*thrust_7;
        torque_y = -0.44906205628078855*thrust_4 + 0.44906205628078855*thrust_5 + 0.44906205628078855*thrust_6 + -0.4490330562807888*thrust_7;
        torque_z = -0.223659*thrust_0 + 0.223659*thrust_1 + -0.4690629221616195*thrust_2 + 0.4690629221616195*thrust_3;

        wrench.force.x = force_x;
        wrench.force.y = force_y;
        wrench.force.z = force_z;
        wrench.torque.x = torque_x;
        wrench.torque.y = torque_y;
        wrench.torque.z = torque_z;

        pub.publish(wrench);
        loop_rate.sleep();
    }

    return 0;
}