#pragma once

#include <iostream>
#include <opencv2/highgui.hpp>

#include "point.h"

using namespace std;
using namespace cv;


class Bone {
public:
	Point3D prev; // base
	Point3D next; // end
	double width; // The average width of the flesh around the bone in millimeters
	Quat rotation;

public:
	Bone() {

	}
	Bone(LEAP_BONE bone) {
		prev = bone.prev_joint;
		next = bone.next_joint;
		width = bone.width;
		rotation = bone.rotation;
	}
};

class Finger {
public:
	int id;
	Bone bones[4];
	int extended; // The distal phalange terminating at the finger tip

public:
	Finger() {

	}
	Finger(LEAP_DIGIT fin) {
		id = fin.finger_id;
		extended = fin.is_extended;
		for (int i = 0; i < 4; i++) {
			bones[i] = fin.bones[i];
		}
	}
};

class Palm {
public:
	Point3D pos; // The center position of the palm in millimeters from the Leap Motion origin
	Point3D normal; // The normal vector to the palm. If your hand is flat, this vector will point downward, or "out" of the front surface of your palm.
	Point3D direction; // The unit direction vector pointing from the palm position toward the fingers.
	Quat orientation; // The quaternion representing the palm's orientation corresponding to the basis{ normal x direction, -normal, -direction }
};

class Frame {
public:
	double grab_angle;
	double grab_strength;
	Palm palm;
	Finger fingers[5];
	double timestamp = 0.0;
public:
	Frame() {

	}
	void setData(double angle, double strength, LEAP_PALM p, LEAP_DIGIT* digits) {
		grab_angle = angle;
		grab_strength = strength;
		palm.pos = p.position;
		for (int i = 0; i < 5; i++) {
			fingers[i] = digits[i];
		}
	}
};

class Image {
public:
	void* data;
	double timestamp = 0.0;
public:
	// aruco 识别 校准
	void display(int width, int height, void* dataA, void* dataB) {
		// cout << "display()" << endl;
		Mat matA = Mat(height, width, CV_8UC1, dataA);
		Mat matB = Mat(height, width, CV_8UC1, dataB);
		imshow("image0", matA);
		imshow("image1", matB);
		waitKey(1);
	}
};