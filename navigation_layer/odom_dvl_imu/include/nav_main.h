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
#include <master_layer/ResetIMU.h>

namespace navigation
{

class NavigationNode
{
public:
    NavigationNode(const ros::NodeHandlePtr &nh);
    ~NavigationNode();

    void Spin();
    void ProcessCartesianPose();
    void PublishData(ros::Time &current_time);
    void BroadcastTransform(Eigen::Vector3d &position, Eigen::Quaterniond &quaternion, ros::Time &current_time);

    bool SetDepthOffsetCallback(odom_dvl_imu::SetDepthOffset::Request &rqst, odom_dvl_imu::SetDepthOffset::Response &response);
    bool SetWorldXYOffsetCallback(odom_dvl_imu::SetWorldXYOffset::Request &rqst, odom_dvl_imu::SetWorldXYOffset::Response &response);

    void FillPoseMsg(Eigen::Vector3d &position, Eigen::Quaterniond &angle, nav_msgs::Odometry &msg);
    void FillTwistMsg(Eigen::Vector3d &linear_velocity, Eigen::Vector3d &angular_velocity, nav_msgs::Odometry &msg);
    void correct_orientation (Eigen::Quaterniond&);
    bool resetCB (master_layer::ResetIMU::Request& req,
                  master_layer::ResetIMU::Response& res);

private:
    ros::NodeHandlePtr nh_;

    ros::Subscriber dvlTwistSubscriber_;
    ros::Subscriber dvlPressureSubscriber_;
    ros::Subscriber imuSubscriber_;

    ros::Publisher navigationOdomPublisher_;
    ros::Publisher odom_pub_;
    tf::TransformBroadcaster odom_broadcaster;

    ros::ServiceServer navigationDepthOffsetServer_;
    ros::ServiceServer navigationXYOffsetServer_;

    DvlData dvlData_;
    IMUData imuData_;

    ExtendedKalmanFilter dvlFilter_;
    Eigen::Vector3d poseEstimation_;
    Eigen::Vector3d localPoseEstimation_;

    double zOffset_;
    double positionFromDepth_;

    Eigen::Quaterniond quaternion_;
    Eigen::Vector3d position_;
    Eigen::Vector3d incrementPosition_;
    Eigen::Vector3d velocity_;
    Eigen::Vector3d angularVelocity_;
    Eigen::Vector3d eulerAngel_;

    Eigen::Vector3d imu_offset_;
    Eigen::Vector3d local_position_;
    Eigen::Quaterniond local_orientation_;
    ros::ServiceServer reset_imu_;
};

} // namespace navigation
#endif //NAVIGATION_NODE_H_