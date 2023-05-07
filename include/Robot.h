
#include <vector>

#include "Utils.h"

class Robot {
public:

	Robot(double robotWidth, double kl, double kr, double camX, double camY);

	bool run(double dt);
	
	double getLeftEncoder() { return m_vl; }
	double getRightEncoder() { return m_vr; }

	double getGyroHeading() { return m_gyroHeading; };

	bool isTagValid() { return m_tagValid; }
	void tagUsed() { m_tagValid = false; }
	double getCamX() { return m_camX; }
	double getCamZ() { return m_camZ; }
	double getTagX() { return m_tagX; }
	double getTagY() { return m_tagY; }

	Pose getPose() { return { m_x, m_y, m_heading }; }

private:

	double ROBOT_WIDTH;
	double KL, KR;
	double CAMERA_X, CAMERA_Y;

	double m_time{0.0};

	struct DriveCmd {
		double vx, omega, time;
	};

	std::vector<DriveCmd> m_driveCmds;

	std::vector<Point> m_tags;

	int m_driveStep{0};
	double m_stepTime{-0.00001};

	double m_x{0.0};
	double m_y{0.0};
	double m_heading{0.0};

	double m_vl;
	double m_vr;

	double m_gyroHeading;

	double m_tagValid;
	double m_camX;
	double m_camZ;
	double m_tagX;
	double m_tagY;
	int m_tagID;

};