#pragma once

struct Pose {
	double x, y, theta;

	Pose operator-(const Pose& other) {
		return { x - other.x, y - other.y, theta - other.theta };
	}
};

struct Point {
	double x, y;
};