#include "Display.h"

#include "Angles.h"

double Display::SCALE = 4.0; // px per in

Display::Display() : 
	m_window(sf::VideoMode(1000, 1000), "KalmanFilterSim"), 
	m_actualRobot(sf::Color::Blue), 
	m_estRobot(sf::Color::Red) {

	m_window.setFramerateLimit(60);

}

void Display::draw(Pose actual, Pose est) {

	sf::Event event;
	while (m_window.pollEvent(event)) {
		if (event.type == sf::Event::Closed)
			m_window.close();
	}
	
	m_window.clear(sf::Color::White);

	m_actualRobot.draw(m_window, actual);
	m_estRobot.draw(m_window, est);

	m_window.display();

}

RobotGraphic::RobotGraphic(sf::Color color) {

	m_shape.setOutlineThickness(3);
	m_shape.setOutlineColor(color);
	m_shape.setSize(sf::Vector2f(36 * Display::SCALE, 36 * Display::SCALE));
	m_shape.setOrigin(sf::Vector2f(36 * Display::SCALE / 2, 36 * Display::SCALE / 2));

}

void RobotGraphic::draw(sf::RenderWindow& window, Pose pose) {

	m_shape.setPosition(sf::Vector2f(pose.x * Display::SCALE + Display::X_OFFSET, pose.y * Display::SCALE + Display::Y_OFFSET));
	m_shape.setRotation(gsu::angles::to_degrees(pose.theta));

	window.draw(m_shape);

}
