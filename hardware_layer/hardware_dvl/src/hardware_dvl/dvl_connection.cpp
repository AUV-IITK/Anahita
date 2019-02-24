#include <dvl_connection.h>

namespace hardware_dvl {

  DVLNode::DVLNode(const ros::NodeHandlePtr &nh) :
      nh_(nh),
      socket_()
      {
        ROS_INFO("Created struct");
        std::string hostname = "192.168.1.101";
        socket_.Connect(hostname, 1033);

        dvl_twist_publisher_ = nh_->advertise<geometry_msgs::TwistStamped>("/dvl/dvl_twist", 1000);
        dvl_fluid_pressure_publisher_ = nh_->advertise<sensor_msgs::FluidPressure>("/dvl/dvl_pressure", 1000);
  }

  DVLNode::~DVLNode() {}

  void DVLNode::Spin() {
    ros::Rate loop_rate(15);  // 15 hz
    ROS_INFO("Inside SPIN");


    while (ros::ok()) {
      ROS_INFO("Inside ROS_OK");

      socket_.Receive();

      dvl_data_ = *(DVLformat21_t*)(socket_.GetRawData());

      if (dvl_data_.header.sync == 0xA5) {
        if (calculateChecksum((unsigned short *) &dvl_data_.data, dvl_data_.header.dataSize)== dvl_data_.header.dataChecksum) 
        {
          timestamp_ = ros::Time::now();
          FillTwistMessage(timestamp_);
          FillFluidPressureMessage(timestamp_);
        }
      }
      ros::spinOnce();
      loop_rate.sleep();
    
    }

  }

  void DVLNode::FillTwistMessage(ros::Time timestamp) {
    geometry_msgs::TwistStamped message;

    message.header.stamp = timestamp;
    message.header.frame_id = "/ENU";

    message.twist.linear.x = dvl_data_.data.velX;
    message.twist.linear.y = dvl_data_.data.velY;
    message.twist.linear.z = dvl_data_.data.velZ1;

    dvl_twist_publisher_.publish(message);
  }

  void DVLNode::FillFluidPressureMessage(ros::Time timestamp) {
    sensor_msgs::FluidPressure message;

    message.header.stamp = timestamp;
    message.header.frame_id = "/ENU";
    message.fluid_pressure = dvl_data_.data.pressure;

    dvl_fluid_pressure_publisher_.publish(message);
  }

} 