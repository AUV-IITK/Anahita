#include "nav_main.h"

namespace navigation
{
    NavigationNode::NavigationNode(const ros::NodeHandlePtr& nh) : nh_(nh), quaternion_(0.0,0.0,0.0,0.0) 
    {
    	dvlTwistSubscriber_ = nh_->subscribe("/anahita/dvl_twist", 100, &DvlData::DvlTwistCallback, &dvlData_);
  		dvlPressureSubscriber_ = nh_->subscribe("/anahita/pressure", 100,&DvlData::DvlPressureCallback, &dvlData_);
    	imuSubscriber_ = nh_->subscribe("/anahita/imu", 100, &IMUData::IMUMsgCallback, &imuData_);

      // navigationDepthOffsetServer_ = nh_->advertiseService("/nav/set_depth_offset", SetDepthOffsetCallback);
      // navigationXYOffsetServer_ = nh_->advertiseService("/nav/set_world_x_y_offset", SetWorldXYOffsetCallback);

      navigationOdomPublisher_ = nh_->advertise<nav_msgs::Odometry>("/anahita/pose_gt", 100);
      position_          = Eigen::Vector3d::Zero();
      incrementPosition_ = Eigen::Vector3d::Zero();
      velocity_          = Eigen::Vector3d::Zero();
      angularVelocity_   = Eigen::Vector3d::Zero();
      eulerAngel_        = Eigen::Vector3d::Zero();
      zOffset_ 		   = 0;
      ROS_INFO("Set up initial constructor successfully");
	}

	NavigationNode::~NavigationNode()
	{
		dvlTwistSubscriber_.shutdown();
		dvlPressureSubscriber_.shutdown();
		imuSubscriber_.shutdown();
		// navigationDepthOffsetServer_.shutdown();
		// navigationXYOffsetServer_.shutdown();
	}
	
	void NavigationNode::Spin()
	{
		ros::Rate loop_rate(15); // 100 hz
		while(ros::ok())
		{
			ros::spinOnce();
			ProcessCartesianPose();
			loop_rate.sleep();
		}
	}

	// bool NavigationNode::SetDepthOffsetCallback(SetDepthOffset::Request &request, SetDepthOffset::Response &response)
	// {
	// 	zOffset_ =this.GetPositionZFromPressure();
	// 	imuData_.SetNewDataReady();
	// 	return true;
	// }

	// bool NavigationNode::SetWorldXYOffsetCallback(SetWorldXYOffset::Request &request, SetWorldXYOffset::Response &response)
	// {
	// 	position_.x() = 0.0f;
	// 	position_.y() = 0.0f;
	// 	dvlData_.SetNewDataReady();
	// 	return true;
	// }

	void NavigationNode::ProcessCartesianPose()
	{
		if (dvlData_.IsNewDataReady() || imuData_.IsNewDataReady())
		{
		    dvlData_.SetNewDataUsed();
		    imuData_.SetNewDataUsed();

		    incrementPosition_ = dvlData_.GetPositionXYZ();
		    positionFromDepth_ = dvlData_.GetPositionZFromPressure();
		    velocity_          = dvlData_.GetVelocityXYZ();
		    angularVelocity_   = imuData_.GetAngularVelocity();
		    eulerAngel_        = imuData_.GetOrientation();
		    quaternion_        = imuData_.GetQuaternion();
			ROS_INFO("Value of incrementPosition_: x: %f, y: %f,z: %f", incrementPosition_.x(), incrementPosition_.y(), incrementPosition_.z());
		    position_ += quaternion_.toRotationMatrix() * incrementPosition_;
			ROS_INFO("(Without EKF) Position.x: %f, Position.y: %f, Position.z: %f, Position.z: %.2f", position_.x(), position_.y(), position_.z());

		    position_.z() = positionFromDepth_ - zOffset_;

            dvlFilter_.Update(position_, poseEstimation_);

			ros::Time currentTime = ros::Time::now();

            PublishData(currentTime);
			BroadcastTransform(poseEstimation_, quaternion_, currentTime);
		}
	}

	void NavigationNode::BroadcastTransform(Eigen::Vector3d &position, Eigen::Quaterniond &quaternion, ros::Time &current_time)
	{
		geometry_msgs::TransformStamped odom_trans;
    	odom_trans.header.stamp = current_time;
    	odom_trans.header.frame_id = "odom";
    	odom_trans.child_frame_id = "base_link";

    	odom_trans.transform.translation.x = position.x();
    	odom_trans.transform.translation.y = position.y();
    	odom_trans.transform.translation.z = position.z();
    	odom_trans.transform.rotation.w = quaternion.w();
		odom_trans.transform.rotation.x = quaternion.x();
		odom_trans.transform.rotation.y = quaternion.y();
		odom_trans.transform.rotation.z = quaternion.z();

    	// send the transform
    	odom_broadcaster.sendTransform(odom_trans);
	}

    void NavigationNode::PublishData(ros::Time &current_time)
    {
        nav_msgs::Odometry odometry_msg;
        odometry_msg.header.frame_id = "odom";
		    odometry_msg.child_frame_id = "base_link";
        odometry_msg.header.stamp = current_time;

        FillPoseMsg(poseEstimation_, eulerAngel_, odometry_msg);
        FillTwistMsg(velocity_, angularVelocity_, odometry_msg);

        navigationOdomPublisher_.publish(odometry_msg);
    }

	void NavigationNode::FillPoseMsg(Eigen::Vector3d &position, Eigen::Quaterniond &quaternion, nav_msgs::Odometry &msg)
	{
		msg.pose.pose.position.x    = position.x();
		msg.pose.pose.position.y    = position.y();
		msg.pose.pose.position.z    = position.z();
		msg.pose.pose.orientation.x = quanternion.x();
		msg.pose.pose.orientation.y = quanternion.y();
		msg.pose.pose.orientation.z = quanternion.z();
		msg.pose.pose.orientation.w = quanternion.w();
		ROS_INFO("Pose Message being filled: Position.x: %f,
				Position.y: %f, Position.z: %f, Angle.x: %f, 
				Angle.y: %f, Angle.z: %f", position.x(), position.y(), 
				position.z(), angle.x(), angle.y(), angle.z());

	}

	void NavigationNode::FillTwistMsg(Eigen::Vector3d &linear_velocity, Eigen::Vector3d &angular_velocity, nav_msgs::Odometry &msg)
	{
		msg.twist.twist.linear.x  = linear_velocity.x();
		msg.twist.twist.linear.y  = linear_velocity.y();
		msg.twist.twist.linear.z  = linear_velocity.z();
		msg.twist.twist.angular.x = angular_velocity.x();
		msg.twist.twist.angular.y = angular_velocity.y();
		msg.twist.twist.angular.z = angular_velocity.z();
		ROS_INFO("Twist Message being filled: Linear.x: %f, 
				Linear.y: %f, Linear.z: %f, AngularVel.x: %f, 
				AngularVel.y: %f, AngularVel.z: %f", linear_velocity.x(), 
				linear_velocity.y(), linear_velocity.z(), angular_velocity.x(), 
				angular_velocity.y(), angular_velocity.z());
	}

}