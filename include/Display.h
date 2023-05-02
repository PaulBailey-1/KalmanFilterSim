#pragma once

#include <SFML/Graphics.hpp>

#include "Pose.h"

class RobotGraphic {
public:

	RobotGraphic(sf::Color color);

	void draw(sf::RenderWindow& window, Pose pose);

private:

	sf::RectangleShape m_shape;

};

class Display {
public:

	static const int X_OFFSET = 300, Y_OFFSET = 300;
	static double SCALE;

	Display();

	void draw(Pose actual, Pose est);

	bool isWindowOpen() { return m_window.isOpen(); }

private:

	sf::RenderWindow m_window;

	RobotGraphic m_actualRobot;
	RobotGraphic m_estRobot;

};