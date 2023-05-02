#include <stdio.h>
#include <SFML/System/Clock.hpp>

#include "Angles.h"
#include "KalmanFilter.h"
#include "Robot.h"
#include "Logger.h"
#include "Pose.h"
#include "Display.h"

int main() {
	
	printf("Initilizing...\n");

	Robot robot;
	KalmanFilter kFilter;

	Logger logger;
	logger.open("log.csv");

	Display display;

	sf::Clock clock;

	double dt = 0.02;

	bool simFinished = false;

	while (clock.getElapsedTime().asSeconds() < 0.0);
	clock.restart();

	printf("Starting simulation\n");
	while (display.isWindowOpen()) {

		if (!simFinished) {
			if (robot.run(dt)) {
				simFinished = true;
				printf("Simulation Finished\n");
				//break;
				continue;
			}

			kFilter.run(dt, robot.getLeftEncoder(), robot.getRightEncoder());

			kFilter.updateWithGyro(robot.getGyroHeading());

			if (robot.isTagValid()) {
				kFilter.updateWithTag(robot.getCamX(), robot.getCamZ(), robot.getTagX(), robot.getTagY());
				robot.tagUsed();
			}

			Pose error = kFilter.getPose() - robot.getPose();
			Logger::log("X Error", error.x);
			Logger::log("Y Error", error.y);
			Logger::log("Theta Error", error.theta);

			logger.nextLine();
		}

		//double dt = clock.getElapsedTime().asSeconds();
		while (clock.getElapsedTime().asSeconds() < dt);
		clock.restart();
		display.draw(robot.getPose(), kFilter.getPose());

	}

	printf("Robot - x:%f, y:%f, th:%f\n", robot.getPose().x, robot.getPose().y, robot.getPose().theta);
	printf("Filter - x:%f, y:%f, th:%f\n", kFilter.getPose().x, kFilter.getPose().y, kFilter.getPose().theta);

	logger.close();

	return 0;
}