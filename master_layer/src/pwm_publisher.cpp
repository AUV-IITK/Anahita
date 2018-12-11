#include <ros/ros.h>
#include <std_msgs/Int32.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "pwm_publisher");
    ros::NodeHandle nh;

    ros::Publisher frontSidewardPublisher = nh.advertise<std_msgs::Int32>("/pwm/sidewardFront", 1000);
    ros::Publisher backSidewardPublisher = nh.advertise<std_msgs::Int32>("/pwm/sidewardBack", 1000);
    ros::Publisher rightForwardPublisher = nh.advertise<std_msgs::Int32>("/pwm/forwardRight", 1000);
    ros::Publisher leftForwardPublisher = nh.advertise<std_msgs::Int32>("/pwm/forwardLeft", 1000);
    ros::Publisher northEastUpwardPublisher = nh.advertise<std_msgs::Int32>("/pwm/upwardNorthEast", 1000);
    ros::Publisher northWestUpwardPublisher = nh.advertise<std_msgs::Int32>("/pwm/upwardNorthWest", 1000);
    ros::Publisher southEastUpwardPublisher = nh.advertise<std_msgs::Int32>("/pwm/upwardSouthEast", 1000);
    ros::Publisher southWestUpwardPublisher = nh.advertise<std_msgs::Int32>("/pwm/upwardSouthWest", 1000);

    std_msgs::Int32 pwm_sideward_front;
    std_msgs::Int32 pwm_sideward_back;
    std_msgs::Int32 pwm_forward_right;
    std_msgs::Int32 pwm_forward_left;
    std_msgs::Int32 pwm_upward_north_east;
    std_msgs::Int32 pwm_upward_north_west;
    std_msgs::Int32 pwm_upward_south_east;
    std_msgs::Int32 pwm_upward_south_west;

    int pwm_sideward_back_straight;
    int pwm_sideward_front_straight;
    // int pwm_sideward_back_turn;
    // int pwm_sideward_front_turn;
    // int pwm_forward_left_turn;
    // int pwm_forward_right_turn;
    int pwm_forward_left_straight;
    int pwm_forward_right_straight;
    // int pwm_upward_back;
    // int pwm_upward_front;
    int pwm_turn;
    int pwm_upward;

    nh.setParam("/pwm_forward_left", 0);
    nh.setParam("/pwm_forward_right", 0);
    // nh.setParam("/pwm_upward_front", 0);
    // nh.setParam("/pwm_upward_back", 0);
    nh.setParam("/pwm_sideward_back_straight", 0);
    nh.setParam("/pwm_sideward_front_straight", 0);
    // nh.setParam("/pwm_sideward_front_turn", 0);
    // nh.setParam("/pwm_sideward_back_turn", 0);
    nh.setParam("/pwm_turn", 0);
    nh.setParam("/pwm_upward", 0);

    ros::Rate r(50);

    while(ros::ok()) {
        // nh.getParam("/pwm_forward_left_turn", pwm_forward_left_turn);
        // nh.getParam("/pwm_forward_right_turn", pwm_forward_right_turn);
        nh.getParam("/pwm_forward_left_straight", pwm_forward_left_straight);
        nh.getParam("/pwm_forward_right_straight", pwm_forward_right_straight);
        // nh.getParam("/pwm_upward_front", pwm_upward_front);
        // nh.getParam("/pwm_upward_back", pwm_upward_back);
        nh.getParam("/pwm_sideward_back_straight", pwm_sideward_back_straight);
        nh.getParam("/pwm_sideward_front_straight", pwm_sideward_front_straight);
        // nh.getParam("/pwm_sideward_front_turn", pwm_sideward_front_turn);
        // nh.getParam("/pwm_sideward_back_turn", pwm_sideward_back_turn);
        nh.getParam("/pwm_turn", pwm_turn);
        nh.getParam("/pwm_upward", pwm_upward);

        pwm_forward_left.data = pwm_forward_left_straight + pwm_turn;
        pwm_forward_right.data = pwm_forward_right_straight - pwm_turn;

        pwm_sideward_back.data = pwm_sideward_back_straight - pwm_turn;
        pwm_sideward_front.data = pwm_sideward_front_straight + pwm_turn;

        pwm_upward_north_east.data = pwm_upward;
        pwm_upward_north_west.data = pwm_upward;

        pwm_upward_south_east.data = pwm_upward;
        pwm_upward_south_west.data = pwm_upward;

        frontSidewardPublisher.publish(pwm_sideward_front);
        backSidewardPublisher.publish(pwm_sideward_back);

        rightForwardPublisher.publish(pwm_forward_right);
        leftForwardPublisher.publish(pwm_forward_left);

        northEastUpwardPublisher.publish(pwm_upward_north_east);
        northWestUpwardPublisher.publish(pwm_upward_north_west);
        southEastUpwardPublisher.publish(pwm_upward_south_east);
        southWestUpwardPublisher.publish(pwm_upward_south_west);

        // std::cout << "----------------------------------" << std::endl;
        // ROS_INFO("PWM forward_right : %d", pwm_forward_right.data);
        // ROS_INFO("PWM forward_left : %d", pwm_forward_left.data);
        // ROS_INFO("PWM sideward_front : %d", pwm_sideward_front.data);
        // ROS_INFO("PWM sideward_back : %d", pwm_sideward_back.data);
        // ROS_INFO("PWM upward_front : %d", pwm_upward_front.data);
        // ROS_INFO("PWM upward_back : %d", pwm_upward_back.data);
        r.sleep();
    }
    return 0;
}
