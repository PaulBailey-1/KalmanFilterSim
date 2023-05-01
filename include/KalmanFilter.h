
#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include "Eigen/Dense"

#include <chrono>
#include <random>

class KalmanFilter {
public:

    KalmanFilter();

    void init(double x, double y, double theta);

    // Extrapolate the state and covariance based on the control
    void run(double dt, double vl, double vr);

    // Update the state and covariance with a measurement
    void updateWithGyro(double heading);
    void updateWithTag(double x, double z, double tagX, double tagY);

    double getXPos() {return m_state(0);}
    double getYPos() {return m_state(1);}
    double getTheta() {return m_state(2);}

private:

    const double ROBOT_WIDTH = 21.0;
    const double KL = -0.099263868, KR = 0.467169095;
    const double CAMERA_X = 12.5, CAMERA_Y = 2.0;

    // The estimated state, x = {fieldX, fieldY, theta}
    Eigen::Vector3d m_state;

    // The covariance matrix, P
    Eigen::Matrix3d m_covariance;

    // The process random noise covariance matrix, Q
    Eigen::Matrix3d m_processNoise;

    double m_gyroVariance;
    Eigen::Matrix2d m_tagCovariance;

    Eigen::Matrix3d m_identity;

};


#endif
