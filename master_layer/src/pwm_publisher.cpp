#include <ros/ros.h>
#include <std_msgs/Int32.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "ros_node");
    ros::NodeHandle nh;

    ros::Publisher frontSidewardPublisher = nh.advertise<std_msgs::Int32>("/pwm/sidewardFront", 1000);
    ros::Publisher backSidewardPublisher = nh.advertise<std_msgs::Int32>("/pwm/sidewardBack", 1000);
    ros::Publisher rightForwardPublisher = nh.advertise<std_msgs::Int32>("/pwm/forwardRight", 1000);
    ros::Publisher leftForwardPublisher = nh.advertise<std_msgs::Int32>("/pwm/forwardLeft", 1000);
    ros::Publisher frontUpwardPublisher = nh.advertise<std_msgs::Int32>("/pwm/upwardFront", 1000);
    ros::Publisher backUpwardPublisher = nh.advertise<std_msgs::Int32>("/pwm/upwardBack", 1000);

    std_msgs::Int32 pwm_sideward_front;
    std_msgs::Int32 pwm_sideward_back;
    std_msgs::Int32 pwm_forward_right;
    std_msgs::Int32 pwm_forward_left;
    std_msgs::Int32 pwm_upward_front;
    std_msgs::Int32 pwm_upward_back;

    int pwm_sideward_back_straight_;
    int pwm_sideward_front_straight_;
    int pwm_sideward_back_turn_;
    int pwm_sideward_front_turn_;
    int pwm_forward_left_;
    int pwm_forward_right_;
    int pwm_upward_back_;
    int pwm_upward_front_;

    while(ros::ok()) {
        nh.getParam("/pwm_forward_left", pwm_forward_left_);
        nh.getParam("/pwm_forward_right", pwm_forward_right_);
        nh.getParam("/pwm_upward_front", pwm_upward_front_);
        nh.getParam("/pwm_upward_back", pwm_upward_back_);
        nh.getParam("/pwm_sideward_back_straight", pwm_sideward_back_straight_);
        nh.getParam("/pwm_sideward_front_straight", pwm_sideward_front_straight_);
        nh.getParam("/pwm_sideward_front_turn", pwm_sideward_front_turn_);
        nh.getParam("/pwm_sideward_back_turn", pwm_sideward_back_turn_);

        pwm_forward_left.data = pwm_forward_left_;
        pwm_forward_right.data = pwm_forward_right_;

        pwm_sideward_back.data = pwm_sideward_back_straight_ + pwm_sideward_back_turn_;
        pwm_sideward_front.data = pwm_sideward_front_straight_ + pwm_sideward_front_turn_;

        pwm_upward_back.data = pwm_upward_back_;
        pwm_upward_front.data = pwm_upward_front_;

        frontSidewardPublisher.publish(pwm_sideward_front);
        backSidewardPublisher.publish(pwm_sideward_back);

        rightForwardPublisher.publish(pwm_forward_right);
        leftForwardPublisher.publish(pwm_forward_left);

        frontUpwardPublisher.publish(pwm_upward_front);
        backUpwardPublisher.publish(pwm_upward_back);

        std::cout << "----------------------------------" << std::endl;
        ROS_INFO("PWM forward_right : %d", pwm_forward_right.data);
        ROS_INFO("PWM forward_left : %d", pwm_forward_left.data);
        ROS_INFO("PWM sideward_front : %d", pwm_sideward_front.data);
        ROS_INFO("PWM sideward_back : %d", pwm_sideward_back.data);
        ROS_INFO("PWM upward_front : %d", pwm_upward_front.data);
        ROS_INFO("PWM upward_back : %d", pwm_upward_back.data);
    }
    return 0;
}