#include "nav_main.h"

namespace navigation
{
    NavigationNode::NavigationNode(const ros::NodeHandlePtr& nh) : nh_(nh), quaternion_(0.0,0.0,0.0,0.0) 
    {
    	dvlTwistSubscriber_ = nh_->subscribe("/anahita/dvl/data", 100, &DvlData::DvlTwistCallback, &dvlData_);
		dvlPressureSubscriber_ = nh_->subscribe("/anahita/pressure", 100,&DvlData::DvlPressureCallback, &dvlData_);
		imuSubscriber_ = nh_->subscribe("/anahita/imu", 100, &IMUData::IMUMsgCallback, &imuData_);

		// navigationDepthOffsetServer_ = nh_->advertiseService("/nav/set_depth_offset", SetDepthOffsetCallback);
		// navigationXYOffsetServer_ = nh_->advertiseService("/nav/set_world_x_y_offset", SetWorldXYOffsetCallback);

		navigationOdomPublisher_ = nh_->advertise<nav_msgs::Odometry>("/odometry", 100);
		position_          = Eigen::Vector3d::Zero();
	    incrementPosition_ = Eigen::Vector3d::Zero();
	    velocity_          = Eigen::Vector3d::Zero();
	    angularVelocity_   = Eigen::Vector3d::Zero();
	    eulerAngel_        = Eigen::Vector3d::Zero();

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
	//         zOffset_ =this.GetPositionZFromPressure();
	//         imuData_.SetNewDataReady();
	//         return true;
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

		    position_ += quaternion_.toRotationMatrix() * incrementPosition_;

		    position_.z() = positionFromDepth_ - zOffset_;

            dvlFilter_.Update(position_, poseEstimation_);

            PublishData();
		}
	}

    void NavigationNode::PublishData()
    {
        nav_msgs::Odometry odometry_msg;
        odometry_msg.header.frame_id = "NED";
        odometry_msg.header.stamp = ros::Time::now();

        FillPoseMsg(poseEstimation_, eulerAngel_, odometry_msg);
        FillTwistMsg(velocity_, angularVelocity_, odometry_msg);

        navigationOdomPublisher_.publish(odometry_msg);
    }

	void NavigationNode::FillPoseMsg(Eigen::Vector3d &position, Eigen::Vector3d &angle, nav_msgs::Odometry &msg)
	{
		msg.pose.pose.position.x    = position.x();
		msg.pose.pose.position.y    = position.y();
		msg.pose.pose.position.z    = position.z();
		msg.pose.pose.orientation.x = angle.x();
		msg.pose.pose.orientation.y = angle.y();
		msg.pose.pose.orientation.z = angle.z();
	}

	void NavigationNode::FillTwistMsg(Eigen::Vector3d &linear_velocity, Eigen::Vector3d &angular_velocity, nav_msgs::Odometry &msg)
	{
		msg.twist.twist.linear.x  = linear_velocity.x();
		msg.twist.twist.linear.y  = linear_velocity.y();
		msg.twist.twist.linear.z  = linear_velocity.z();
		msg.twist.twist.angular.x = angular_velocity.x();
		msg.twist.twist.angular.y = angular_velocity.y();
		msg.twist.twist.angular.z = angular_velocity.z();
	}

}