#include <stdio.h>
#include <SFML/System/Clock.hpp>

#include "Angles.h"
#include "KalmanFilter.h"
#include "Robot.h"
#include "Logger.h"
#include "Utils.h"
#include "Display.h"

const double ROBOT_WIDTH = 21.0;
//const double KL = sqrt(0.004742407), KR = sqrt(0.006080527);
const double KL = sqrt(0.1), KR = sqrt(0.1);
const double CAMERA_X = 0.0, CAMERA_Y = 0.0;

int main() {
	
	printf("Initilizing...\n");

	Robot robot(ROBOT_WIDTH, KL, KR, CAMERA_X, CAMERA_Y);
	KalmanFilter kFilter(ROBOT_WIDTH, KL, KR, CAMERA_X, CAMERA_Y);

	Logger logger;
	logger.open("log.csv");

	Display display;

	sf::Clock clock;

	double dt = 0.02;

	bool simFinished = false;

	clock.restart();
	double time = 0.0;

	int trials = 1;
	int trial = 1;
	int in = 0;
	double avgErr = 0.0;

	bool running = true;

	printf("Starting simulation\n");
	while (running) {

		if (!simFinished) {
			if (robot.run(dt)) {
				simFinished = true;
				printf("Simulation %i Finished\n", trial);
				//break;
				continue;
			}

			kFilter.run(dt, robot.getLeftEncoder(), robot.getRightEncoder());

			kFilter.updateWithGyro(robot.getGyroHeading());

			//if (robot.isTagValid()) {
			//	kFilter.updateWithTag(robot.getCamX(), robot.getCamZ(), robot.getTagX(), robot.getTagY());
			//	robot.tagUsed();
			//}

			Pose error = kFilter.getPose() - robot.getPose();
			Logger::log("X Error", error.x);
			Logger::log("Y Error", error.y);
			Logger::log("Theta Error", gsu::angles::to_degrees(error.theta));
			Logger::log("Pos Error", sqrt(error.x * error.x + error.y * error.y));

		} else {

			//simFinished = false;

			//if (trial == trials) {
			//	running = false;
			//	simFinished = true;
			//	printf("Total in: %i\nSuccess: %f\n", in, (double) in / trials * 100);
			//	avgErr /= (trials);
			//	printf("Average Error Out: %f\n", avgErr);
			//} else {
			//	trial++;
			//}

			//Pose error = kFilter.getPose() - robot.getPose();
			//if (abs(error.x) < 2 * sqrt(kFilter.getCov()(0, 0)) && abs(error.y) < 2 * sqrt(kFilter.getCov()(1, 1))) {
			//	in++;
			//	avgErr += sqrt(error.x * error.x + error.y * error.y);

			//} else {
			//	avgErr += sqrt(error.x * error.x + error.y * error.y);
			//}

			//robot = Robot(ROBOT_WIDTH, KL, KR, CAMERA_X, CAMERA_Y);
			//kFilter = KalmanFilter(ROBOT_WIDTH, KL, KR, CAMERA_X, CAMERA_Y);

		}

		double dt = clock.getElapsedTime().asSeconds();
		while (clock.getElapsedTime().asSeconds() < dt );
		time += clock.getElapsedTime().asSeconds();
		clock.restart();
		display.draw(robot.getPose(), kFilter.getPose(), kFilter.getCov());
		running = display.isWindowOpen();

		logger.nextLine();

	}

	printf("Robot - x:%f, y:%f, th:%f\n", robot.getPose().x, robot.getPose().y, robot.getPose().theta);
	printf("Filter - x:%f, y:%f, th:%f\n", kFilter.getPose().x, kFilter.getPose().y, kFilter.getPose().theta);

	logger.close();

	return 0;
}