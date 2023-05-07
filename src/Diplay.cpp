#include "Display.h"

#include "Angles.h"
#include "Logger.h"

double Display::SCALE = 4.0; // px per in

Display::Display() : 
	m_window(sf::VideoMode(1000, 1000), "KalmanFilterSim"), 
	m_actualRobot(sf::Color::Blue), 
	m_estRobot(sf::Color::Red),
	m_errorEllipse() {

	m_window.setFramerateLimit(60);

	m_tags.push_back({ 40, -20, 0 });
	m_tags.push_back({ 100, 40, 90 });
	m_tags.push_back({ 40, 100, 0 });
	m_tags.push_back({ -20, 40, 90 });

}

void Display::draw(Pose actual, Pose est, Eigen::Matrix3d cov) {

	sf::Event event;
	while (m_window.pollEvent(event)) {
		if (event.type == sf::Event::Closed)
			m_window.close();
	}
	
	m_window.clear(sf::Color::White);

	for (TagGraphic tag : m_tags) {
		m_window.draw(tag.shape);
	}

	m_actualRobot.draw(m_window, actual);
	m_estRobot.draw(m_window, est);

	m_errorEllipse.draw(m_window, est, cov);

	m_window.display();

}

RobotGraphic::RobotGraphic(sf::Color color) {

	m_shape.setFillColor(sf::Color::Transparent);
	m_shape.setOutlineThickness(3);
	m_shape.setOutlineColor(color);
	m_shape.setSize(sf::Vector2f(24 * Display::SCALE, 24 * Display::SCALE));
	m_shape.setOrigin(sf::Vector2f(24 * Display::SCALE / 2, 24 * Display::SCALE / 2));

	m_dot.setFillColor(color);
	m_dot.setOutlineColor(color);
	m_dot.setRadius(1 * Display::SCALE);
	m_dot.setOrigin(sf::Vector2f(1 * Display::SCALE, 1 * Display::SCALE));


}

void RobotGraphic::draw(sf::RenderWindow& window, Pose pose) {

	m_shape.setPosition(sf::Vector2f(pose.x * Display::SCALE + Display::X_OFFSET, -pose.y * Display::SCALE + Display::Y_OFFSET));
	m_shape.setRotation(gsu::angles::to_degrees(-pose.theta));

	m_dot.setPosition(m_shape.getPosition());

	window.draw(m_shape);
	window.draw(m_dot);

}

TagGraphic::TagGraphic(double x, double y, double rot) {
	shape.setFillColor(sf::Color::Black);
	shape.setSize(sf::Vector2f(20, 2));
	shape.setOrigin(sf::Vector2f(10, 1));
	shape.setPosition(sf::Vector2f(x * Display::SCALE + Display::X_OFFSET, -y * Display::SCALE + Display::Y_OFFSET));
	shape.setRotation(rot);
}

ErrorEllipseGraphic::ErrorEllipseGraphic() {
	m_shape.setFillColor(sf::Color::Transparent);
	m_shape.setOutlineColor(sf::Color::Red);
	m_shape.setOutlineThickness(3);
	m_shape.setPointCount(100);
}

void ErrorEllipseGraphic::draw(sf::RenderWindow& window, Pose pose, Eigen::Matrix3d covFull) {

	Eigen::Matrix2d cov = covFull.block<2, 2>(0, 0);
	Eigen::EigenSolver<Eigen::Matrix2d>solver(cov);
	Eigen::Vector2d eigenValues = solver.eigenvalues().real();
	Eigen::Matrix2d eigenVectors = solver.eigenvectors().real();

	Eigen::Vector2d ellipseFunction;
	
	for (int i = 0; i < m_shape.getPointCount(); i++) {
		double t = i * 2 * M_PI / m_shape.getPointCount();

		int s = 2;
		ellipseFunction << s * sqrt(eigenValues(0)) * cos(t), s * sqrt(eigenValues(1)) * sin(t);
		ellipseFunction = eigenVectors * ellipseFunction;
		ellipseFunction *= Display::SCALE;
		ellipseFunction(0) += pose.x * Display::SCALE + Display::X_OFFSET;
		ellipseFunction(1) += -pose.y * Display::SCALE + Display::Y_OFFSET;
		m_shape.setPoint(i, sf::Vector2f(ellipseFunction(0), ellipseFunction(1)));
	}

	window.draw(m_shape);

	Logger::log("Cov1", sqrt(eigenValues(0)));
	Logger::log("Cov2", sqrt(eigenValues(1)));
	Logger::log("Cov", sqrt(4 * eigenValues(0) + 4 * eigenValues(1)));

}