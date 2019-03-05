#pragma once

#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include "point.h"

using namespace std;
using namespace cv;

#define markersX 1
#define markersY 1
#define markersLen 0.127              
#define markersSep 0.02
#define markersDic 0

Vec3d _rvec;
Vec3d _tvec;

class Bone {
public:
	Point3D prev; // base
	Point3D next; // end

	Mat realPrev;
	Mat realNext;

public:
	Bone() {

	}
	Bone(LEAP_BONE bone) {
		prev = bone.prev_joint;
		next = bone.next_joint;
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

	Mat realPos;
	Mat realNormal;
	Mat realDirection;
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
		palm.normal = p.normal;
		palm.direction = p.direction;
		for (int i = 0; i < 5; i++) {
			fingers[i] = digits[i];
		}
	}
	Mat calc(Point3D point, int flag) {
		cv::Mat rMat;
		cv::Rodrigues(_rvec, rMat);
		Vec4d pos(-point.x, point.z, point.y, 1); // 若识别到朝上
		cv::Mat T = cv::Mat::eye(4, 4, rMat.type()); // T is 4x4
		T(cv::Range(0, 3), cv::Range(0, 3)) = rMat * 1; // copies R into T
		if (flag) {
			T(cv::Range(0, 3), cv::Range(3, 4)) = Mat(_tvec) * 1000; // copies tvec into T
		}
		T = T.inv();
		Mat posMat = Mat(pos);
		Mat ans = T * posMat;
		return ans;
	}
	void convert() {
		palm.realPos = calc(palm.pos, 1);
		palm.realNormal = calc(palm.normal, 0);
		palm.realDirection = calc(palm.direction, 0);
	
		/*
		double temp = palm.realDirection.at<double>(0, 0) * palm.realDirection.at<double>(0, 0)
			+ palm.realDirection.at<double>(1, 0) * palm.realDirection.at<double>(1, 0)
			+ palm.realDirection.at<double>(2, 0) * palm.realDirection.at<double>(2, 0);
		*/
		// convert fingers

		for (int i = 0; i < 5; i++) {
			// std::cout << fingers[i].id << std::endl;
			for (int j = 0; j < 4; j++) {
				fingers[i].bones[j].realNext = calc(fingers[i].bones[j].next, 1);
				fingers[i].bones[j].realPrev = calc(fingers[i].bones[j].prev, 1);
				// std::cout << fingers[i].bones[j].realNext << std::endl << fingers[i].bones[j].realPrev << std::endl;
			}
			// std::cout << std::endl;
		}

	}
};

class Image {
public:
	// Vec3d _rvec;
	// Vec3d _tvec;
	void* data;
	double timestamp = 0.0;
public:
	int readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
		FileStorage fs(filename, FileStorage::READ);
		if (!fs.isOpened())
			return 0;
		fs["camera_matrix"] >> camMatrix;
		fs["distortion_coefficients"] >> distCoeffs;
		return 1;
	}

	// aruco 识别 校准
	void display(int width, int height, void* dataA, void* dataB) {
		// cout << "display()" << endl;
		Mat image = Mat(height, width, CV_8UC1, dataA);
		Mat matB = Mat(height, width, CV_8UC1, dataB);
		// imshow("image0", image);
		// imshow("image1", matB);
		// waitKey(1);
		// 只取matA做detect
		
		int minColor = 255;
		int maxColor = 0;
		for (int r = 0; r < image.rows; r++) {
			for (int c = 0; c < image.cols; c++) {
				int color = image.at<uchar>(r, c);
				minColor = min(minColor, color);
				maxColor = max(maxColor, color);
			}
		}
		double span = 255.0 / (maxColor - minColor);
		for (int r = 0; r < image.rows; r++) {
			for (int c = 0; c < image.cols; c++) {
				image.at<uchar>(r, c) = (image.at<uchar>(r, c) - minColor) * span;
			}
		}
		image *= 2;
		
		Mat imageCopy;

		vector< int > ids;
		vector< vector< Point2f > > corners, rejected;
		Vec3d rvec, tvec;

		Ptr<aruco::Dictionary> dictionary =
			aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(markersDic));

		float axisLength = 0.5f * ((float)min(markersX, markersY) * (markersLen + markersSep)
			+ markersSep);
		Ptr<aruco::GridBoard> gridboard =
			aruco::GridBoard::create(markersX, markersY, markersLen, markersSep, dictionary);
		Ptr<aruco::Board> board = gridboard.staticCast<aruco::Board>();

		Mat camMatrix, distCoeffs;
		int readOk = readCameraParameters(std::string("calibrate.txt"), camMatrix, distCoeffs);
		if (readOk == 0) {
			cerr << "Invalid camera file" << endl;
			return;
		}

		Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
		detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX; // do corner refinement in ma

		// detect markers
		aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
		aruco::refineDetectedMarkers(image, board, corners, ids, rejected);

		// estimate board pose
		int markersOfBoardDetected = 0;
		if (ids.size() == 1)
			markersOfBoardDetected =
			aruco::estimatePoseBoard(corners, ids, board, camMatrix, distCoeffs, rvec, tvec);

		// draw results
		image.copyTo(imageCopy);
		if (ids.size() == 1) {
			aruco::drawDetectedMarkers(imageCopy, corners, ids);
		}

		/*
		if (rejected.size() > 0)
			aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));
			*/
		if (markersOfBoardDetected == 1)
			aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvec, tvec, axisLength);
		//if (ids.size() == 1)
		if (ids.size() == 1) {
			_rvec = rvec;
			_tvec = tvec;
		}

			imshow("out", imageCopy);
		waitKey(1);
	}
};