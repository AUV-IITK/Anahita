#include <ros/ros.h>
#include <hyperion_msgs/Thrust.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "pwm_publisher");
    ros::NodeHandle nh;

    ros::Publisher pwmPublisher = nh.advertise<hyperion_msgs::Thrust>("/pwm", 1000);

//     ros::Publisher frontSidewardPublisher = nh.advertise<std_msgs::Int32>("/pwm/sidewardFront", 1000);
//     ros::Publisher backSidewardPublisher = nh.advertise<std_msgs::Int32>("/pwm/sidewardBack", 1000);
//     ros::Publisher rightForwardPublisher = nh.advertise<std_msgs::Int32>("/pwm/forwardRight", 1000);
//     ros::Publisher leftForwardPublisher = nh.advertise<std_msgs::Int32>("/pwm/forwardLeft", 1000);
//     ros::Publisher northEastUpwardPublisher = nh.advertise<std_msgs::Int32>("/pwm/upwardNorthEast", 1000);
//     ros::Publisher northWestUpwardPublisher = nh.advertise<std_msgs::Int32>("/pwm/upwardNorthWest", 1000);
//     ros::Publisher southEastUpwardPublisher = nh.advertise<std_msgs::Int32>("/pwm/upwardSouthEast", 1000);
//     ros::Publisher southWestUpwardPublisher = nh.advertise<std_msgs::Int32>("/pwm/upwardSouthWest", 1000);

//     std_msgs::Int32 pwm_sideward_front;
//     std_msgs::Int32 pwm_sideward_back;
//     std_msgs::Int32 pwm_forward_right;
//     std_msgs::Int32 pwm_forward_left;
//     std_msgs::Int32 pwm_upward_north_east;
//     std_msgs::Int32 pwm_upward_north_west;
//     std_msgs::Int32 pwm_upward_south_east;
//     std_msgs::Int32 pwm_upward_south_west;

    int pwm_sway;
    int pwm_surge;
    int pwm_turn;
    int pwm_roll;
    int pwm_pitch;
    int pwm_heave;

    int kill_signal = 0;

    nh.setParam("/pwm_sway", 0);
    nh.setParam("/pwm_surge", 0);
    nh.setParam("/pwm_heave", 0);
    nh.setParam("/pwm_turn", 0);
    nh.setParam("/pwm_roll", 0);
    nh.setParam("/pwm_pitch", 0);

    nh.setParam("/kill_signal", 0);

    ros::Rate r(50);
    
    hyperion_msgs::Thrust pwm;

    while(ros::ok()) {

        nh.getParam("/pwm_surge", pwm_surge);
        nh.getParam("/pwm_heave", pwm_heave);
        nh.getParam("/pwm_sway", pwm_sway);

        nh.getParam("/pwm_turn", pwm_turn);
        nh.getParam("/pwm_roll", pwm_roll);
        nh.getParam("/pwm_pitch", pwm_pitch);

	nh.getParam("/kill_signal", kill_signal);

        pwm.forward_left = -pwm_surge - pwm_turn;
        pwm.forward_right = -pwm_surge + pwm_turn;

        pwm.sideward_back = -pwm_sway + pwm_turn;
        pwm.sideward_front = -pwm_sway - pwm_turn;

        pwm.upward_north_east = -pwm_heave - pwm_roll - pwm_pitch;
        pwm.upward_north_west = -pwm_heave + pwm_roll + pwm_pitch;

        pwm.upward_south_east = -pwm_heave - pwm_roll + pwm_pitch;
        pwm.upward_south_west = -pwm_heave + pwm_roll - pwm_pitch;

	if (kill_signal) {

            ROS_INFO("KILL SIGNAL RECIEVED === %d ++++++++++++++ ", kill_signal);
            
            pwm.forward_left = 0;
            pwm.forward_right = 0;
            
            pwm.sideward_back = 0;
            pwm.sideward_front = 0;
            
            pwm.upward_north_east = 0;
            pwm.upward_north_west = 0;
            
            pwm.upward_south_east = 0;
            pwm.upward_south_west = 0;
	}
	
        // frontSidewardPublisher.publish(pwm_sideward_front);
        // backSidewardPublisher.publish(pwm_sideward_back);

        // rightForwardPublisher.publish(pwm_forward_right);
        // leftForwardPublisher.publish(pwm_forward_left);

        // northEastUpwardPublisher.publish(pwm_upward_north_east);
        // northWestUpwardPublisher.publish(pwm_upward_north_west);
        // southEastUpwardPublisher.publish(pwm_upward_south_east);
        // southWestUpwardPublisher.publish(pwm_upward_south_west);
	
	pwmPublisher.publish(pwm);

        std::cout << "----------------------------------" << std::endl;
        ROS_INFO("PWM forward_right : %d", pwm.forward_right);
        ROS_INFO("PWM forward_left : %d", pwm.forward_left);
        ROS_INFO("PWM sideward_front : %d", pwm.sideward_front);
        ROS_INFO("PWM sideward_back : %d", pwm.sideward_back);
	ROS_INFO("PWM upward north east: %d", pwm.upward_north_east);
	ROS_INFO("PWM upward south east: %d", pwm.upward_south_east);
	ROS_INFO("PWM upward north west: %d", pwm.upward_north_west);
	ROS_INFO("PWM upward south west: %d", pwm.upward_south_west);
        r.sleep();
    }
    return 0;
}
