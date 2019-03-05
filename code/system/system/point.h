#pragma once

class Point3D {
public:
	double x, y, z;

public:
	Point3D() {

	}
	Point3D(double _x, double _y, double _z) {
		x = _x;
		y = _y;
		z = _z;
	}
	Point3D(LEAP_VECTOR v) {
		x = v.x;
		y = v.y;
		z = v.z;
	}
};

class Quat {
public:
	double x, y, z, w;

public:
	Quat() {

	}
	Quat(LEAP_QUATERNION q) {
		x = q.x;
		y = q.y;
		z = q.z;
		w = q.w;
	}
};
