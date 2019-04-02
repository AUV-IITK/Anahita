#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/FluidPressure.h>

double depth_data = 0;

void pressureCB (const sensor_msgs::FluidPressure _msg) {
    depth_data = (_msg.fluid_pressure - 101.325) / 9.80638;
}

int main (int argc, char** argv) {
    ros::init(argc, argv, "pressure_publisher");
    ros::NodeHandle nh;

    ros::Subscriber pressure_sub = nh.subscribe<sensor_msgs::FluidPressure>("/anahita/pressure", 1000, &pressureCB);
    ros::Publisher depth_pub = nh.advertise<std_msgs::Float32>("/anahita/z_coordinate", 1000);

    bool enable_pressure = false;
    std_msgs::Float32 depth;

    bool set_reference_depth = false;

    ros::Rate loop_rate(50);

    while (ros::ok()) {
        depth.data = depth_data;
        depth_pub.publish(depth);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
