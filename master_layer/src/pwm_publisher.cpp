#include <ros/ros.h>
#include <anahita_msgs/Thrust.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "pwm_publisher");
    ros::NodeHandle nh;

    ros::Publisher pwmPublisher = nh.advertise<anahita_msgs::Thrust>("/pwm", 1);

    int pwm_sway;
    int pwm_surge;
    int pwm_yaw;
    int pwm_roll;
    int pwm_pitch;
    int pwm_heave;

    int torpedo;
    int marker_dropper;

    bool kill_signal = false;

    nh.setParam("/pwm_sway", 0);
    nh.setParam("/pwm_surge", 0);
    nh.setParam("/pwm_heave", 0);
    nh.setParam("/pwm_yaw", 0);
    nh.setParam("/pwm_roll", 0);
    nh.setParam("/pwm_pitch", 0);

    nh.setParam("/kill_signal", false);

    nh.setParam("/torpedo", 0);
    nh.setParam("/marker_dropper", 0);

    ros::Rate r(20);

    anahita_msgs::Thrust pwm;

    while(ros::ok()) {
        nh.getParam("/pwm_surge", pwm_surge);
        nh.getParam("/pwm_heave", pwm_heave);
        nh.getParam("/pwm_sway", pwm_sway);

        nh.getParam("/pwm_yaw", pwm_yaw);
        nh.getParam("/pwm_roll", pwm_roll);
        nh.getParam("/pwm_pitch", pwm_pitch);

    	nh.getParam("/kill_signal", kill_signal);

        nh.getParam("/marker_dropper", marker_dropper);
        nh.getParam("/torpedo", torpedo);

        pwm.forward_left = -pwm_surge - pwm_yaw;
        pwm.forward_right = -pwm_surge + pwm_yaw;

        pwm.sideward_back = -pwm_sway + pwm_yaw;
        pwm.sideward_front = -pwm_sway - pwm_yaw;

        pwm.upward_north_east = -pwm_heave - pwm_roll - pwm_pitch;
        pwm.upward_north_west = -pwm_heave + pwm_roll + pwm_pitch;

        pwm.upward_south_east = -pwm_heave - pwm_roll + pwm_pitch;
        pwm.upward_south_west = -pwm_heave + pwm_roll - pwm_pitch;

        pwm.torpedo = torpedo;
        pwm.marker_dropper = marker_dropper;

        if (kill_signal) {

            ROS_INFO("KILL SIGNAL RECIEVED");
            
            nh.setParam("/pwm_sway", 0);
            nh.setParam("/pwm_surge", 0);
            nh.setParam("/pwm_heave", 0);
            nh.setParam("/pwm_yaw", 0);
            nh.setParam("/pwm_roll", 0);
            nh.setParam("/pwm_pitch", 0);

            nh.setParam("/kill_signal", false);

            nh.setParam("/marker_dropper", 0);
            nh.setParam("/torpedo", 0);
        }
            
        pwmPublisher.publish(pwm);

        r.sleep();
    }
    return 0;
}
