#ifndef NAV_MAIN_H_
#define NAV_MAIN_H_

#include <ros/ros.h>
#include <memory>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <odom_dvl_imu/SetDepthOffset.h>
#include <odom_dvl_imu/SetWorldXYOffset.h>

#include "dvl_data.h"
#include "imu_data.h"
#include "ekf.h"

namespace navigation{

    class NavigationNode {
        public:

            NavigationNode(const ros::NodeHandlePtr &nh);
            ~NavigationNode();

            void Spin();
            void ProcessCartesianPose();
            void PublishData(ros::Time &current_time);
        	void BroadcastTransform(Eigen::Vector3d &position, Eigen::Quaterniond &quaternion, ros::Time &current_time);

        private:
        
            // bool SetDepthOffsetCallback(SetDepthOffset::Request &rqst, SetDepthOffset::Response &response);
            // bool SetWorldXYOffsetCallback(SetWorldXYOffset::Request &rqst,SetWorldXYOffset::Response &response);

            void FillPoseMsg(Eigen::Vector3d &position, Eigen::Quaterniond &angle, nav_msgs::Odometry &msg);
            void FillTwistMsg(Eigen::Vector3d &linear_velocity, Eigen::Vector3d &angular_velocity, nav_msgs::Odometry &msg);

            ros::NodeHandlePtr nh_;

            ros::Subscriber dvlTwistSubscriber_;
            ros::Subscriber dvlPressureSubscriber_;
            ros::Subscriber imuSubscriber_;

            ros::Publisher  navigationOdomPublisher_;
            tf::TransformBroadcaster odom_broadcaster;

            // ros::ServiceServer navigationDepthOffsetServer_;
            // ros::ServiceServer navigationXYOffsetServer_;

            DvlData dvlData_;
            IMUData imuData_;

            ExtendedKalmanFilter dvlFilter_;
            Eigen::Vector3d      poseEstimation_;

            double zOffset_;
            double positionFromDepth_;

            Eigen::Quaterniond quaternion_;
            Eigen::Vector3d    position_;
            Eigen::Vector3d    incrementPosition_;
            Eigen::Vector3d    velocity_;
            Eigen::Vector3d    angularVelocity_;
            Eigen::Vector3d    eulerAngel_;

    };

}
#endif //NAVIGATION_NODE_H_