#include <dvl.h>

namespace hardware_dvl {

  DVLNode::DVLNode(const ros::NodeHandlePtr &nh) :
      nh_(nh),
      socket_()
      {
        std::string hostname = "192.168.1.240";
        socket_.Connect(hostname, 9002);

        dvl_twist_publisher_ = nh_->advertise<geometry_msgs::TwistStamped>("/provider_dvl/dvl_twist", 1000);
        dvl_fluid_pressure_publisher_ = nh_->advertise<sensor_msgs::FluidPressure>("/provider_dvl/dvl_pressure", 1000);
  }

  DVLNode::~DVLNode() {}

  void DVLNode::Spin() {
    ros::Rate loop_rate(15);  // 15 hz

    while (ros::ok()) {
      ros::spinOnce();

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