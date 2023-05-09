
// https://ecse.monash.edu/centres/irrc/LKPubs/MECSE-1995-1.pdf
// http://www.cs.unc.edu/~tracker/media/pdf/SIGGRAPH2001_CoursePack_08.pdf

#include "KalmanFilter.h"
#include "Angles.h"
#include "Logger.h"

#include <math.h>
#include <iostream>

KalmanFilter::KalmanFilter(double robotWidth, double kl, double kr, double camX, double camY) {

    ROBOT_WIDTH = robotWidth;
    KL = kl;
    KR = kr;
    CAMERA_X = camX;
    CAMERA_Y = camY;

    m_state = {0.0, 0.0, 0.0};
    m_covariance.setIdentity();
    m_covariance *= 1E-5;
    
    m_gyroVariance = 3e-6;
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

    //m_state(2) += omega * dt;

    // std::cout << "KF:: new state = \n" << m_state << std::endl;

    // -- Extrapolate the covariance --

    // Determine the process noise, Q

    static double klrSum = KL * KL + KR * KR;
    static double krlDiff = KR * KR - KL * KL;
    //double e = 0.3;

    if (std::abs(vl - vr) < 30) { // Traveling in straight line
        double d = v * dt;
        //m_processNoise.setZero();
        //m_processNoise(0, 0) = pow(d * e / 10, 2);
        //m_processNoise(1, 1) = pow(d * e, 2);
        m_processNoise << klrSum / 4, (d * krlDiff) / (4 * ROBOT_WIDTH), krlDiff / (2 * ROBOT_WIDTH),
                    0, (d * d * klrSum) / (3 * ROBOT_WIDTH * ROBOT_WIDTH), (d * klrSum) / (2 * ROBOT_WIDTH * ROBOT_WIDTH),
                    0, 0, klrSum / (ROBOT_WIDTH * ROBOT_WIDTH);

        m_processNoise(1,0) = m_processNoise(0,1);
        m_processNoise(2,0) = m_processNoise(0,2);
        m_processNoise(2,1) = m_processNoise(1,2);
        m_processNoise *= std::abs(d);

        //m_processNoise(2, 2) = 0.00000000;

    } else { // Rotating in place
        double a = omega * dt;
        //m_processNoise.setZero();
        //m_processNoise(2, 2) = pow(a * e, 2);
        m_processNoise << (ROBOT_WIDTH * klrSum * std::abs(sin(a) + a)) / 16, 0, 0,
                            0, klrSum * (ROBOT_WIDTH / 2) * std::abs((a - sin(a)) / 2), 0,
                            0, 0, (std::abs(a) / (2 * ROBOT_WIDTH)) * klrSum;

        m_processNoise(2, 2) = 0.0001;
    }

    double rotateBy = -m_state(2);
    Eigen::Matrix3d rotation;
    rotation <<  cos(rotateBy), -sin(rotateBy), 0,
                sin(rotateBy), cos(rotateBy), 0,
                0, 0, 1;

    m_processNoise = rotation * m_processNoise * rotation.transpose();
    
    // Define the jacobian of f, the extrapolation function

    Eigen::Matrix3d F;
    F <<    1, 0, abs(-v * dt * sin(m_state(2))),
            0, 1, abs(v * dt * cos(m_state(2))),
            0, 0, 1;
    
    double p22 = m_covariance(2, 2);
    m_covariance = F * m_covariance * F.transpose() + m_processNoise;
    if (m_covariance(2, 2) - p22 < 0) {
        printf("");
    }

    // std::cout << "KF:: new covariance = \n" << m_covariance << std::endl;

//    std::cout << "KF:: Q = \n" << Q << std::endl;

    Logger::log("xEst", m_state(0));
    Logger::log("yEst", m_state(1));
    Logger::log("headingEst", m_state(2));
    Logger::log("p00", m_covariance(0, 0));
    Logger::log("p11", m_covariance(1, 1));
    Logger::log("p22", m_covariance(2, 2));

}


void KalmanFilter::updateWithGyro(double heading) {

    //std::cout << "Update with gyro\n";

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

    //std::cout << "Update with tag\n";

    // Measurement vector, z
    Eigen::Vector2d z;
    z << camX, camZ;

    // Observation function h (already evaluated)
    Eigen::Vector2d h;
    h << CAMERA_Y + cos(m_state(2)) * (m_state(1) - tagY) + sin(m_state(2)) * (tagX - m_state(0)), // x
         -CAMERA_X + cos(m_state(2)) * (tagX - m_state(0)) + sin(m_state(2)) * (tagY - m_state(1)); // z

    // The Jacobian matrix of h with respect to the state
    Eigen::Matrix<double, 2, 3> H;
    H << -sin(m_state(2)), cos(m_state(2)), -sin(m_state(2)) * (m_state(1) - tagY) + cos(m_state(2)) * (tagX - m_state(0)),
         -cos(m_state(2)), -sin(m_state(2)), -sin(m_state(2)) * (tagX - m_state(0)) + cos(m_state(2)) * (tagY - m_state(1));

    double d = sqrt(camX * camX + camZ * camZ);
    double xVar = (d / 150) * 4 + 1;
    xVar *= xVar;
    double zVar = (d / 150) * 2 + 1;
    zVar *= zVar;
    m_tagCovariance << xVar, 0,
                       0, zVar;

    Eigen::Matrix<double, 3, 2> K = m_covariance * H.transpose() * (H * m_covariance * H.transpose() + m_tagCovariance).inverse();

    // std::cout << "KF:: z = \n" << z << std::endl;
    // std::cout << "KF:: K = \n" << K << std::endl;

    m_state += K * (z - h);
    m_covariance = (m_identity - K * H) * m_covariance;

    // std::cout << "KF:: state = \n" << m_state << std::endl;
    // std::cout << "KF:: covariance = \n" << m_covariance << std::endl;

}