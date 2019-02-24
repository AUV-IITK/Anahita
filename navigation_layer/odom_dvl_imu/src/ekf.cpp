#include "ekf.h"

namespace navigation{

    ExtendedKalmanFilter::ExtendedKalmanFilter()
    {
        Initialization(0.1, 1 ^ -4, 0.1);
    }

    void ExtendedKalmanFilter::Initialization(float pval, float qval, float rval)
    {
        previousNoise_.setIdentity();
        postNoise_.setIdentity();
        jacobiansTransition_.setIdentity();
        jacobiansMeasurement_.setIdentity();
        processNoise_.setIdentity();
        measurementNoise_.setIdentity();

        previousNoise_    *= pval;
        processNoise_     *= qval;
        measurementNoise_ *= rval;
    }


    void ExtendedKalmanFilter::Update(Eigen::Vector3d &measurement, Eigen::Vector3d &estimation)
    {
        Eigen::Matrix3d inverse_matrix;
        Eigen::Matrix3d gain;
        Eigen::Vector3d stateEstimation = measurement;

        previousNoise_ = jacobiansTransition_ * postNoise_ * jacobiansTransition_ + processNoise_;

        inverse_matrix = jacobiansMeasurement_ * previousNoise_ * jacobiansMeasurement_ + measurementNoise_;

        inverse_matrix.inverse();

        gain = previousNoise_ * jacobiansMeasurement_ * inverse_matrix;

        stateEstimation += gain * (measurement - stateEstimation);

        estimation = stateEstimation;
    }
}