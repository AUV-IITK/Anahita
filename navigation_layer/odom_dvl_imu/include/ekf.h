#ifndef EKF_H_
#define EKF_H_

#include <ros/ros.h>
#include <memory>
#include <eigen3/Eigen/Geometry>
namespace navigation{

    class ExtendedKalmanFilter
    {
    public:
       
        ExtendedKalmanFilter();
        ~ExtendedKalmanFilter() = default;

        void Update(Eigen::Vector3d &measurement, Eigen::Vector3d &estimation);

        void Initialization(float pval, float qval, float rval);

    private:
        Eigen::Matrix3d previousNoise_;
        Eigen::Matrix3d postNoise_;
        Eigen::Matrix3d jacobiansTransition_;
        Eigen::Matrix3d jacobiansMeasurement_;
        Eigen::Matrix3d processNoise_;
        Eigen::Matrix3d measurementNoise_;
    };
}
#endif // EKF_H_