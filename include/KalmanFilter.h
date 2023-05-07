
#pragma once

#include "Utils.h"

#include "Eigen/Dense"

#include <chrono>
#include <random>

class KalmanFilter {
public:

    KalmanFilter(double robotWidth, double kl, double kr, double camX, double camY);

    void init(double x, double y, double theta);

    // Extrapolate the state and covariance based on the control
    void run(double dt, double vl, double vr);

    // Update the state and covariance with a measurement
    void updateWithGyro(double heading);
    void updateWithTag(double x, double z, double tagX, double tagY);

    double getXPos() {return m_state(0);}
    double getYPos() {return m_state(1);}
    double getTheta() {return m_state(2);}

    Pose getPose() { return { m_state(0), m_state(1), m_state(2) }; }
    Eigen::Matrix3d getCov() { return m_covariance; }

private:

    double ROBOT_WIDTH;
    double KL, KR;
    double CAMERA_X, CAMERA_Y;

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
