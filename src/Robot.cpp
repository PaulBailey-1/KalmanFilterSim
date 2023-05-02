#include <random>
#include "Logger.h"

#include <Robot.h>

Robot::Robot() {

	const double PI = 3.1415926535;
	double speed = 80;
	double distance = 80;
	m_driveCmds.push_back({ speed, 0.0, distance / speed }); // vx, omega, time
	m_driveCmds.push_back({0.0, PI, 0.5}); // vx, omega, time
	m_driveCmds.push_back({ speed, 0.0, distance / speed }); // vx, omega, time
	m_driveCmds.push_back({ 0.0, PI, 0.5 }); // vx, omega, time
	m_driveCmds.push_back({ speed, 0.0, distance / speed }); // vx, omega, time
	m_driveCmds.push_back({ 0.0, PI, 0.5 }); // vx, omega, time
	m_driveCmds.push_back({ speed, 0.0, distance / speed }); // vx, omega, time

}

bool Robot::run(double dt) {

	m_time += dt;
	m_stepTime += dt;
	Logger::log("time", m_time);

	if (m_driveStep > m_driveCmds.size() - 1) {
		return false;
	}
	DriveCmd target = m_driveCmds[m_driveStep];
	if (m_stepTime > target.time) {
		m_stepTime = 0.00001;
		m_driveStep++;
		if (m_driveStep > m_driveCmds.size() - 1) {
			return true;
		}
		target = m_driveCmds[m_driveStep];
	}

	m_x += target.vx * cos(m_heading) * dt;
	m_y += target.vx * sin(m_heading) * dt;

	m_heading += target.omega * dt;

	static std::default_random_engine generator;

	m_vl = target.vx - target.omega * ROBOT_WIDTH / 2;
	double dl = m_vl * dt;
	std::normal_distribution<double> leftEncoderDistribution(dl, sqrt(abs(KL * KL * dl)));
	m_vl = leftEncoderDistribution(generator) / dt;

	m_vr = target.vx + target.omega * ROBOT_WIDTH / 2;
	double dr = m_vr * dt;
	std::normal_distribution<double> rightEncoderDistribution(dr, sqrt(abs(KR * KR * dr)));
	m_vr = rightEncoderDistribution(generator) / dt;

	std::normal_distribution<double> gyroDistribution(m_heading, sqrt(0.001));
	m_gyroHeading = gyroDistribution(generator);

	double camFX = 0.0;
	double camFY = 0.0;
	if (fmod(m_time, 1.0) < dt * 2 && m_time > dt * 3) {
		m_tagValid = true;

		m_tagX = 0.0;
		m_tagY = 0.0; // TODO

		//m_camX = CAMERA_Y + m_y + m_tagX * sin(m_heading) - m_tagY * cos(m_heading), // x
		//m_camZ = -CAMERA_X - m_x + m_tagX * cos(m_heading) + m_tagY * sin(m_heading); // z

		m_camX = CAMERA_Y + cos(m_heading) * (m_y - m_tagY) + sin(m_heading) * (m_tagX - m_x); // x
		m_camZ = -CAMERA_X + cos(m_heading) * (m_tagX - m_x) + sin(m_heading) * (m_tagY - m_y); // z

		std::normal_distribution<double> camXDistribution(m_camX, sqrt(25.0));
		m_camX = camXDistribution(generator);
		std::normal_distribution<double> camZDistribution(m_camZ, sqrt(9.0));
		m_camZ = camZDistribution(generator);

		camFX = m_tagX - cos(m_heading) * (CAMERA_X + m_camZ) + sin(m_heading) * (m_camX - CAMERA_Y);
		camFY = m_tagY + cos(m_heading) * (m_camX - CAMERA_Y) - sin(m_heading) * (CAMERA_X + m_camZ);
	}

	Logger::log("robotX", m_x);
	Logger::log("robotY", m_y);
	Logger::log("robotHeading", m_heading);
	Logger::log("vl", m_vl);
	Logger::log("vr", m_vr);
	Logger::log("gyroHeading", m_gyroHeading);
	Logger::log("camX", m_camX);
	Logger::log("camZ", m_camZ);
	Logger::log("camFX", camFX);
	Logger::log("camFY", camFY);

	return false;
}