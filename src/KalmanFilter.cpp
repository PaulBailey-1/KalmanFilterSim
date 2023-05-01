
// https://ecse.monash.edu/centres/irrc/LKPubs/MECSE-1995-1.pdf
// http://www.cs.unc.edu/~tracker/media/pdf/SIGGRAPH2001_CoursePack_08.pdf

#include "KalmanFilter.h"
#include "Angles.h"

#include <math.h>
#include <iostream>

KalmanFilter::KalmanFilter() {

    m_state = {0.0, 0.0, 0.0};
    m_covariance.setIdentity();
    m_covariance *= 1E-5;
    
    m_gyroVariance = 0.0001;
    m_tagCovariance << 25.0, 0.0,
                       0.0, 9.0;

    m_identity.setIdentity();
}

void KalmanFilter::init(double x, double y, double theta) {
    m_state  << x, y, theta;
}

void KalmanFilter::run(double dt, double vl, double vr) {

    // -- Extrapolate the state from the control --

    // vl and vr compose the control vector, u
    double v = (vl + vr) / 2.0;
    double omega = (vr - vl) / ROBOT_WIDTH;

    double theta = m_state(2);

    double dx = v * dt * cos(theta);
    double dy = v * dt * sin(theta);

    // double dd = sqrt(dx * dx + dy * dy);

    // std::cout << "KF:: state = \n" << m_state << std::endl;

    m_state(0) += dx;
    m_state(1) += dy;

    m_state(2) += omega * dt;

    // std::cout << "KF:: new state = \n" << m_state << std::endl;

    // -- Extrapolate the covariance --

    // Determine the process noise, Q

    static double klrSum = KL * KL + KR * KR;
    static double krlDiff = KR * KR - KL * KL;

    if (std::abs(vl - vr) < 10) { // Traveling in straight line
        double d = v * dt;
            m_processNoise << klrSum / 4, (d * krlDiff) / (4 * ROBOT_WIDTH), krlDiff / (2 * ROBOT_WIDTH),
                        0, (d * d * klrSum) / (3 * ROBOT_WIDTH * ROBOT_WIDTH), (d * klrSum) / (2 * ROBOT_WIDTH * ROBOT_WIDTH),
                        0, 0, klrSum / (ROBOT_WIDTH * ROBOT_WIDTH);

            m_processNoise(1,0) = m_processNoise(0,1);
            m_processNoise(2,0) = m_processNoise(0,2);
            m_processNoise(2,1) = m_processNoise(1,2);
            m_processNoise *= std::abs(d);
    } else { // Rotating in place
        double a = omega * dt;
        m_processNoise << (ROBOT_WIDTH * klrSum * std::abs(sin(a) + a)) / 16, m_processNoise(0,1), m_processNoise(0,2),
                            m_processNoise(1,0), klrSum * (ROBOT_WIDTH / 2) * std::abs((a - sin(a)) / 2), m_processNoise(1,2),
                            m_processNoise(2,0), m_processNoise(2,1), (std::abs(a) / (2 * ROBOT_WIDTH)) * klrSum;
    }

    double rotateBy = -m_state(2);
    Eigen::Matrix3d rotation;
    rotation <<  cos(rotateBy), -sin(rotateBy), 0,
                sin(rotateBy), cos(rotateBy), 0,
                0, 0, 1;

    m_processNoise = rotation * m_processNoise * rotation.transpose();
    
    // Define the jacobian of f, the extrapolation function

    Eigen::Matrix3d F; 
    F <<    1, 0, -v * dt * sin(theta),
            0, 1, v * dt * cos(theta),
            0, 0, 1;
    
    m_covariance = F * m_covariance * F.transpose() + m_processNoise;

    // std::cout << "KF:: new covariance = \n" << m_covariance << std::endl;

//    std::cout << "KF:: Q = \n" << Q << std::endl;

}


void KalmanFilter::updateWithGyro(double heading) {

    std::cout << "Update with gyro\n";

    // Measurement vector, z
    double z = heading;

    // The observation matrix, H
    Eigen::Matrix<double, 1, 3> H;
    H << 0, 0, 1;

    Eigen::Matrix<double, 3, 1> K;
    K << 0, 0, m_covariance(2,2) / (m_covariance(2,2) + m_gyroVariance);

    // std::cout << "KF:: z = \n" << z << std::endl;
    // std::cout << "KF:: K = \n" << K << std::endl;

    m_state += K * (z - H * m_state);
    m_covariance = (m_identity - K * H) * m_covariance;

    // std::cout << "KF:: state = \n" << m_state << std::endl;
    // std::cout << "KF:: covariance = \n" << m_covariance << std::endl;

}

void KalmanFilter::updateWithTag(double camX, double camZ, double tagX, double tagY) {

    std::cout << "Update with tag\n";

    // Measurement vector, z
    Eigen::Vector2d z;
    z << camX, camZ;

    // Observation function h (already evaluated)
    Eigen::Vector2d h;
    h << CAMERA_Y + m_state(1) + tagX * sin(m_state(2)) - tagY * cos(m_state(2)), // x
         -CAMERA_X - m_state(0) + tagX * cos(m_state(2)) + tagY * sin(m_state(2)); // z

    // The Jacobian matrix of h with respect to the state
    Eigen::Matrix<double, 2, 3> H;
    H << 0, 1, tagX * cos(m_state(2)) + tagY * sin(m_state(2)),
         1, 0, -tagX * sin(m_state(2)) + tagY * cos(m_state(2));

    Eigen::Matrix<double, 3, 2> K = m_covariance * H.transpose() * (H * m_covariance * H.transpose() + m_tagCovariance).inverse();

    // std::cout << "KF:: z = \n" << z << std::endl;
    // std::cout << "KF:: K = \n" << K << std::endl;

    m_state += K * (z - h);
    m_covariance = (m_identity - K * H) * m_covariance;

    // std::cout << "KF:: state = \n" << m_state << std::endl;
    // std::cout << "KF:: covariance = \n" << m_covariance << std::endl;

}