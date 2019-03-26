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
#define markersLen 0.1973             
#define markersSep 0.01
#define markersDic 0

class Frame;

Vec3d _rvec;
Vec3d _tvec;
vector<Frame> frames;

double curIndex_x = 0;
double lastIndex_x = 0;
double curIndex_y = 0;
double lastIndex_y = 0;
double curIndex_z = 0;
double lastIndex_z = 0;

double curMiddle_x = 0;
double lastMiddle_x = 0;
double curMiddle_y = 0;
double lastMiddle_y = 0;
double curMiddle_z = 0;
double lastMiddle_z = 0;

int first = 0;
int _first = 0;
int second = 0;
int _second = 0;
int third = 0;
int _third = 0;
int fourth = 0;
int _fourth = 0;

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

double index_x, index_y, index_z;
double palm_x, palm_y, palm_z;
double fin_x[5][5], fin_y[5][5], fin_z[5][5];

class Frame {
public:
	double grab_angle;
	double grab_strength;
	Palm palm;
	Finger fingers[5];
	long long timestamp;
	Vec3d tv;
	Vec3d rv;
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
		tv = _tvec;
		rv = _rvec;
	}
	Mat calc(Point3D point, int flag) {
		if (flag) {
			point.x -= 25;
		}
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
		ans.at<double>(2, 0) = ans.at<double>(2, 0) - T.at<double>(2, 3) * 0.22 + 40;
		return ans;
	}
	void convert() {
		palm.realPos = calc(palm.pos, 1);
		palm.realNormal = calc(palm.normal, 0);
		palm.realDirection = calc(palm.direction, 0);

		palm_x = palm.pos.x - 25;
		palm_y = palm.pos.y;
		palm_z = palm.pos.z;
	
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

				if (j == 0) {
					fin_x[i][j] = fingers[i].bones[j].prev.x - 25;
					fin_y[i][j] = fingers[i].bones[j].prev.y;
					fin_z[i][j] = fingers[i].bones[j].prev.z;
				}

				fin_x[i][j + 1] = fingers[i].bones[j].next.x - 25;
				fin_y[i][j + 1] = fingers[i].bones[j].next.y;
				fin_z[i][j + 1] = fingers[i].bones[j].next.z;

				if (i == 1 && j == 3) {
					index_x = fingers[i].bones[j].next.x - 25;
					index_y = fingers[i].bones[j].next.y;
					index_z = fingers[i].bones[j].next.z;
					cv::Mat rMat;
					cv::Rodrigues(_rvec, rMat);
					// printf("%lf\n", fingers[i].bones[j].realNext.at<double>(2, 0));
					//std::cout << fingers[i].bones[j].realNext << std::endl;
				}
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
	std::queue<int> rects;
public:
	Image() {
		std::cout << "Image()" << std::endl;
		int last = -1;
		for (int i = 0; i < 6; i++) {
			int temp = rand() % 4;
			while (temp == last) {
				temp = rand() % 4;
			}
			last = temp;
			rects.push(temp);
		}
	}
	int readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
		FileStorage fs(filename, FileStorage::READ);
		if (!fs.isOpened())
			return 0;
		fs["camera_matrix"] >> camMatrix;
		fs["distortion_coefficients"] >> distCoeffs;
		return 1;
	}

	// aruco 识别 校准
	int display(int width, int height, void* dataA, void* dataB) {

		// cout << "display()" << endl;
		Mat image = Mat(height, width, CV_8UC1, dataA);
		//Mat matB = Mat(height, width, CV_8UC1, dataB);
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
		//image *= 2;
		
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
			return -1;
		}


		Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();

		// detect markers
		try {
			detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_CONTOUR;
			aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
		} catch (Exception e) {
			std::cout << "Haha" << std::endl;
			detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;
			aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
		}
		aruco::refineDetectedMarkers(image, board, corners, ids, rejected);

		static int error_cnt = 0;
		static float error_t = 0.05;

		// estimate board pose
		image.copyTo(imageCopy);
		if (ids.size() == 1) {
			aruco::estimatePoseBoard(corners, ids, board, camMatrix, distCoeffs, rvec, tvec);
			if ((abs(_rvec[0] - rvec[0]) < error_t && abs(_rvec[1] - rvec[1]) < error_t && abs(_rvec[2] - rvec[2]) < error_t) || error_cnt >= 5) {
				aruco::drawDetectedMarkers(imageCopy, corners, ids);
				_rvec = rvec;
				_tvec = tvec;
				error_cnt = 0;
			}
			else {
				error_cnt++;
			}
		}
		// std::cout << _rvec << " " << _tvec << std::endl;
		// [2.21748, -2.18703, 0.083776] [0.254348, 0.109625, 0.211468]
		_rvec[0] = 2.21748;
		_rvec[1] = -2.18703;
		_rvec[2] = 0.083776;
		_tvec[0] = 0.254348;
		_tvec[1] = 0.109625;
		_tvec[2] = 0.211468;
		aruco::drawAxis(imageCopy, camMatrix, distCoeffs, _rvec, _tvec, axisLength);


		// 画出游戏界面

		cv::line(imageCopy, cv::Point(250, 0), cv::Point(250, 240), cv::Scalar(0, 0, 0), 2);
		cv::line(imageCopy, cv::Point(300, 0), cv::Point(300, 240), cv::Scalar(0, 0, 0), 2);
		cv::line(imageCopy, cv::Point(350, 0), cv::Point(350, 240), cv::Scalar(0, 0, 0), 2);
		cv::line(imageCopy, cv::Point(400, 0), cv::Point(400, 240), cv::Scalar(0, 0, 0), 2);
		cv::line(imageCopy, cv::Point(450, 0), cv::Point(450, 240), cv::Scalar(0, 0, 0), 2);

		cv::rectangle(imageCopy, cv::Point(252, 0), cv::Point(298, 240), cv::Scalar(255, 0, 0), -1);
		cv::rectangle(imageCopy, cv::Point(302, 0), cv::Point(348, 240), cv::Scalar(255, 0, 0), -1);
		cv::rectangle(imageCopy, cv::Point(352, 0), cv::Point(398, 240), cv::Scalar(255, 0, 0), -1);
		cv::rectangle(imageCopy, cv::Point(402, 0), cv::Point(448, 240), cv::Scalar(255, 0, 0), -1);

		for (int i = 0; i < 6; i++) {
			int t = rects.front();
			rects.pop();
			rects.push(t);

			cv::rectangle(imageCopy, cv::Point(252 + t * 50, (3 - i) * 40), cv::Point(298 + t * 50, (2 - i) * 40), cv::Scalar(0, 0, 0), -1);
		}

		// 根据串口数据传入model获取ring的touch

		// 根据pos计算cv上的touch
		double arg1 = ((double*)camMatrix.data)[0];
		double arg2 = ((double*)camMatrix.data)[2];
		double arg3 = ((double*)camMatrix.data)[4];
		double arg4 = ((double*)camMatrix.data)[5];

		// cv::Point(-fin_x[1][3] / fin_y[1][3] * arg1 + arg2, fin_z[1][3] / fin_y[1][3] * arg3 + arg4);

		// std::cout << -fin_x[1][3] / fin_y[1][3] * arg1 + arg2 << std::endl;

		if (frames.size() > 0) {
			Frame current = frames.back();

			curIndex_x = current.fingers[1].bones[3].realNext.at<double>(0);
			curIndex_y = current.fingers[1].bones[3].realNext.at<double>(1);
			curIndex_z = current.fingers[1].bones[3].realNext.at<double>(2);

			curMiddle_x = current.fingers[2].bones[3].realNext.at<double>(0);
			curMiddle_y = current.fingers[2].bones[3].realNext.at<double>(1);
			curMiddle_z = current.fingers[2].bones[3].realNext.at<double>(2);

			// std::cout << curIndex_x << std::endl;
			// std::cout << curIndex_z << std::endl;
			// std::cout << std::endl;
			double fingerPos = -fin_x[1][3] / fin_y[1][3] * arg1 + arg2;

			if (first == 0 && lastIndex_z > 10.0 && curIndex_z < 10.0 && fingerPos < 300.0 && fingerPos > 250.0) {
				first = 1;
			}
			else if (first == 1 && lastIndex_z < 10.0 && curIndex_z > 10.0) {
				first = 0;
				_first = 1;
				std::cout << "first" << std::endl;
			}
			if (second == 0 && lastIndex_z > 10.0 && curIndex_z < 10.0 && fingerPos < 350.0 && fingerPos > 300.0) {
				second = 1;
			}
			else if (second == 1 && lastIndex_z < 10.0 && curIndex_z > 10.0) {
				second = 0;
				_second = 1;
				std::cout << "second" << std::endl;
			}
			if (third == 0 && lastIndex_z > 10.0 && curIndex_z < 10.0 && fingerPos < 400.0 && fingerPos > 350.0) {
				third = 1;
			}
			else if (third == 1 && lastIndex_z < 10.0 && curIndex_z > 10.0) {
				third = 0;
				_third = 1;
				std::cout << "third" << std::endl;
			}
			if (fourth == 0 && lastIndex_z > 10.0 && curIndex_z < 10.0 && fingerPos < 450.0 && fingerPos > 400.0) {
				fourth = 1;
			}
			else if (fourth == 1 && lastIndex_z < 10.0 && curIndex_z > 10.0) {
				fourth = 0;
				_fourth = 1;
				std::cout << "fourth" << std::endl;
			}

			lastIndex_x = curIndex_x;
			lastIndex_y = curIndex_y;
			lastIndex_z = curIndex_z;
		}

		// 游戏逻辑

		if (_first == 1 && rects.front() == 0) {
			_first = 0;
			rects.pop();
			int last = rects.back();
			int temp = rand() % 4;
			while (temp == last) {
				temp = rand() % 4;
			}
			rects.push(temp);
		}
		else if (second == 1 && rects.front() == 1) {
			_second = 0;
			rects.pop();
			int last = rects.back();
			int temp = rand() % 4;
			while (temp == last) {
				temp = rand() % 4;
			}
			rects.push(temp);
		}
		else if (third == 1 && rects.front() == 2) {
			_third = 0;
			rects.pop();
			int last = rects.back();
			int temp = rand() % 4;
			while (temp == last) {
				temp = rand() % 4;
			}
			rects.push(temp);
		}
		else if (fourth == 1 && rects.front() == 3) {
			_fourth = 0;
			rects.pop();
			int last = rects.back();
			int temp = rand() % 4;
			while (temp == last) {
				temp = rand() % 4;
			}
			rects.push(temp);
		}

		// std::cout << _rvec << std::endl;

		cv::circle(imageCopy, cv::Point(-palm_x / palm_y * arg1 + arg2, palm_z / palm_y * arg3 + arg4), 2, cv::Scalar(0, 255, 255), 2);

		for (int i = 0; i < 5; i++) {
			for (int j = 0; j < 5; j++) {
				cv::circle(imageCopy, cv::Point(-fin_x[i][j] / fin_y[i][j] * arg1 + arg2, fin_z[i][j] / fin_y[i][j] * arg3 + arg4), 2, cv::Scalar(0, 0, 255), 2);
				
				if (j > 0 && j < 4) {
					cv::line(imageCopy, 
						cv::Point(-fin_x[i][j] / fin_y[i][j] * arg1 + arg2, fin_z[i][j] / fin_y[i][j] * arg3 + arg4),
						cv::Point(-fin_x[i][j + 1] / fin_y[i][j + 1] * arg1 + arg2, fin_z[i][j + 1] / fin_y[i][j + 1] * arg3 + arg4), 
						cv::Scalar(128, 128, 128),
						2);
				}

				if (j == 0 && (i == 0 || i == 1 || i == 4)) {
					cv::line(imageCopy,
						cv::Point(-fin_x[i][j] / fin_y[i][j] * arg1 + arg2, fin_z[i][j] / fin_y[i][j] * arg3 + arg4),
						cv::Point(-fin_x[i][j + 1] / fin_y[i][j + 1] * arg1 + arg2, fin_z[i][j + 1] / fin_y[i][j + 1] * arg3 + arg4),
						cv::Scalar(128, 128, 128),
						2);
				}	
			}
		}

		for (int i = 1; i < 4; i++) {
			cv::line(imageCopy,
				cv::Point(-fin_x[i][0] / fin_y[i][0] * arg1 + arg2, fin_z[i][0] / fin_y[i][0] * arg3 + arg4),
				cv::Point(-fin_x[i + 1][0] / fin_y[i + 1][0] * arg1 + arg2, fin_z[i + 1][0] / fin_y[i + 1][0] * arg3 + arg4),
				cv::Scalar(128, 128, 128),
				2);
			cv::line(imageCopy,
				cv::Point(-fin_x[i][1] / fin_y[i][1] * arg1 + arg2, fin_z[i][1] / fin_y[i][1] * arg3 + arg4),
				cv::Point(-fin_x[i + 1][1] / fin_y[i + 1][1] * arg1 + arg2, fin_z[i + 1][1] / fin_y[i + 1][1] * arg3 + arg4),
				cv::Scalar(128, 128, 128),
				2);
		}

		Mat dst = Mat::zeros(480, 640, CV_8UC3);
		resize(imageCopy, dst, dst.size());

		imshow("out", dst);

		char key = (char)waitKey(1);
		if (key == 's') {
			// start
			return 1;
		}
		else if (key == 'n') {
			// next
			return 2;
		}
		else if (key == 'r') {
			// redo
			return 3;
		}
		else if (key == 'q') {
			// quit
			return 4;
		}
		else {
			return -1;
		}
	}
};