#include <stdio.h>
#include <Angles.h>

#include <KalmanFilter.h>
#include <Robot.h>
#include "Logger.h"

int main() {
	
	printf("Initilizing...\n");

	Robot robot;
	KalmanFilter kFilter;

	Logger logger;
	logger.open("log.csv");

	printf("Starting simulation\n");
	while (true) {

		if (robot.run(0.02)) {
			break;
		}

		//kFilter.run(0.02, robot.getLeftEncoder(), robot.getRightEncoder());

		//kFilter.updateWithGyro(robot.getGyroHeading());

		//if (robot.isTagValid()) {
		//	kFilter.updateWithTag(robot.getCamX(), robot.getCamZ(), robot.getTagX(), robot.getTagY());
		//}


		logger.nextLine();
	}

	logger.close();

	printf("Stopped\n");

	return 0;
}