#pragma once

#include <SFML/Graphics.hpp>
#include <Eigen/dense>

#include "Utils.h"

class RobotGraphic {
public:

	RobotGraphic(sf::Color color);

	void draw(sf::RenderWindow& window, Pose pose);

private:

	sf::RectangleShape m_shape;
	sf::CircleShape m_dot;

};

struct TagGraphic {
	TagGraphic(double x, double y, double rot);

	sf::RectangleShape shape;
};

class ErrorEllipseGraphic {
public:
	ErrorEllipseGraphic();

	void draw(sf::RenderWindow& window, Pose pose, Eigen::Matrix3d covFull);

private:

	sf::ConvexShape m_shape;

};

class Display {
public:

	static const int X_OFFSET = 300, Y_OFFSET = 700;
	static double SCALE;

	Display();

	void draw(Pose actual, Pose est, Eigen::Matrix3d cov);

	bool isWindowOpen() { return m_window.isOpen(); }

private:

	sf::RenderWindow m_window;

	RobotGraphic m_actualRobot;
	RobotGraphic m_estRobot;

	ErrorEllipseGraphic m_errorEllipse;

	std::vector<TagGraphic> m_tags;

};