#include <random>
#include "Logger.h"

#include <Robot.h>

const double PI = 3.1415926535;

Robot::Robot(double robotWidth, double kl, double kr, double camX, double camY) {

	ROBOT_WIDTH = robotWidth;
	KL = kl;
	KR = kr;
	CAMERA_X = camX;
	CAMERA_Y = camY;

	double speed = 80;
	double distance = 80;
	m_driveCmds.push_back({ speed, 0.0, distance / speed }); // vx, omega, time
	m_driveCmds.push_back({0.0, PI, 0.5}); // vx, omega, time
	m_driveCmds.push_back({ speed, 0.0, distance / speed }); // vx, omega, time
	m_driveCmds.push_back({ 0.0, PI, 0.5 }); // vx, omega, time
	m_driveCmds.push_back({ speed, 0.0, distance / speed }); // vx, omega, time
	m_driveCmds.push_back({ 0.0, PI, 0.5 }); // vx, omega, time
	m_driveCmds.push_back({ speed, 0.0, distance / speed }); // vx, omega, time

	m_tags.push_back({ 40, -20 });
	m_tags.push_back({ 100, 40 });
	m_tags.push_back({ 40, 100 });
	m_tags.push_back({-20, 40});

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

	static std::random_device rd;
	static std::default_random_engine generator(rd());
	//static int seed = 9272;
	//seed++;
	//generator.seed(seed);

	m_vl = target.vx - target.omega * ROBOT_WIDTH / 2;
	double dl = m_vl * dt;
	std::normal_distribution<double> leftEncoderDistribution(dl, sqrt(abs(KL * KL * dl)));
	m_vl = leftEncoderDistribution(generator) / dt;

	m_vr = target.vx + target.omega * ROBOT_WIDTH / 2;
	double dr = m_vr * dt;
	std::normal_distribution<double> rightEncoderDistribution(dr, sqrt(abs(KR * KR * dr)));
	m_vr = rightEncoderDistribution(generator) / dt;

	std::normal_distribution<double> gyroDistribution(m_heading, sqrt(3e-6));
	m_gyroHeading = gyroDistribution(generator);

	if (m_y > 70 && m_x < 50) {
		printf("");
	}

	double camFX = 0.0;
	double camFY = 0.0;
	double a = 0.0;
	for (int i = 0; i < m_tags.size(); i++) {

		Point tag = m_tags[i];
		double d = sqrt(pow(tag.x - m_x, 2) + pow(tag.y - m_y, 2));
		a = -atan2(tag.y - m_y, tag.x - m_x) + m_heading;

		if (gsu::angles::normalize_angle_positive(atan2(tag.y - m_y, tag.x - m_x) - m_heading) < PI / 4) {
			m_tagValid = true;
			m_tagID = i;

			m_tagX = tag.x;
			m_tagY = tag.y;

			m_camX = d * sin(a);
			m_camZ = d * cos(a);

			//m_camX = CAMERA_Y + cos(m_heading) * (m_y - m_tagY) + sin(m_heading) * (m_tagX - m_x); // x
			//m_camZ = -CAMERA_X + cos(m_heading) * (m_tagX - m_x) + sin(m_heading) * (m_tagY - m_y); // z

			double xVar = (d / 150) * 4 + 1;
			double zVar = (d / 150) * 2 + 1;

			std::normal_distribution<double> camXDistribution(m_camX, xVar);
			m_camX = camXDistribution(generator);
			std::normal_distribution<double> camZDistribution(m_camZ, zVar);
			m_camZ = camZDistribution(generator);

			camFX = m_tagX - cos(m_heading) * (CAMERA_X + m_camZ) + sin(m_heading) * (CAMERA_Y - m_camX);
			camFY = m_tagY + cos(m_heading) * (m_camX - CAMERA_Y) - sin(m_heading) * (CAMERA_X + m_camZ);
		}
	}
	

	Logger::log("robotX", m_x);
	Logger::log("robotY", m_y);
	Logger::log("robotHeading", m_heading);
	Logger::log("vl", m_vl);
	Logger::log("vr", m_vr);
	Logger::log("gyroHeading", m_gyroHeading);
	Logger::log("camX", m_camX);
	Logger::log("camZ", m_camZ);
	Logger::log("camAngle", a);
	Logger::log("tagID", m_tagID);
	Logger::log("camFX", camFX);
	Logger::log("camFY", camFY);

	return false;
}