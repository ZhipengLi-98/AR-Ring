#undef __cplusplus

#include <stdio.h>
#include <stdlib.h>
#include "LeapC.h"
#include "ExampleConnection.h"

#include "frame.h"
#include <vector>
#include <sys/timeb.h>
#include <fstream>
#include <iostream>
#include <string>

// 垂直 0, 水平 1
#define oritention 0

int cnt = 0;
int flag = 0;

vector<Frame> frames;
string name;

long long start;

static LEAP_CONNECTION* connectionHandle;

void writeFrames(string fileName);

long long getSystemTime() {
	timeb t;
	ftime(&t);
	return t.time * 1000 + t.millitm;
}

/** Callback for when the connection opens. */
static void OnConnect() {
	printf("Connected.\n");
}

/** Callback for when a device is found. */
static void OnDevice(const LEAP_DEVICE_INFO *props) {
	printf("Found device %s.\n", props->serial);
}

/** Callback for when a frame of tracking data is available. */
static void OnFrame(const LEAP_TRACKING_EVENT *frame) {
	// printf("Frame %lli with %i hands.\n", (long long int)frame->info.frame_id, frame->nHands);

	static int theOnlyHand = -1;

	// 默认下应该只有一只手
	// assert frame->nHands == 1
	if (true) {
		// std::cout << getSystemTime() << std::endl;
		for (uint32_t h = 0; h < frame->nHands; h++) {
			Frame next;
			LEAP_HAND* hand = &frame->pHands[h];
			if (frame->nHands == 1 || hand->id == theOnlyHand) {
				theOnlyHand = hand->id;
				next.setData(hand->grab_angle, hand->grab_strength, hand->palm, hand->digits);
				next.convert();
				next.timestamp = getSystemTime();
				frames.push_back(next);
			}
			/*
			printf("    Hand id %i is a %s hand with position (%f, %f, %f).\n",
				hand->id,
				(hand->type == eLeapHandType_Left ? "left" : "right"),
				hand->palm.position.x,
				hand->palm.position.y,
				hand->palm.position.z);
				*/

		}
	}
}

void printMsg() {
	switch (cnt) {
	case 0:
		std::cout << "食指指腹 放松" << std::endl;
		break;
	case 1:
		std::cout << "食指指腹 握拳" << std::endl;
		break;
	case 2:
		std::cout << "双指指腹 放松" << std::endl;
		break;
	case 3:
		std::cout << "双指指腹 握拳" << std::endl;
		break;
	case 4:
		std::cout << "食指指尖 放松" << std::endl;
		break;
	case 5:
		std::cout << "食指指尖 握拳" << std::endl;
		break;
	case 6:
		std::cout << "中指指腹 放松" << std::endl;
		break;
	case 7:
		std::cout << "食指叩击 握拳" << std::endl;
		break;
	case 8:
		std::cout << "双指指尖 放松" << std::endl;
		break;
	case 9:
		std::cout << "三指指腹 放松" << std::endl;
		break;
	}
}

#include <omp.h>

static void solve_distortion(unsigned char* in_raw, float* in_distortion, int in_distortion_width) {
	float destinationWidth = 640;
	float destinationHeight = 240;
	//unsigned char destination[(int)destinationWidth][(int)destinationHeight];
	unsigned char destination[640 * 240];

	//define needed variables outside the inner loop

	const unsigned char* raw = in_raw;
	const float* distortion_buffer = in_distortion;

	//Local variables for values needed in loop
	const int distortionWidth = in_distortion_width;
	//const int width = image.width();
	//const int height = image.height();
	const int width = 640;
	const int height = 240;

#pragma omp parallel for
	for (int i = 0; i < (int)destinationWidth; i++) {
		float calibrationX, calibrationY;
		float weightX, weightY;
		int denormalizedX, denormalizedY;
		float dX, dX1, dX2, dX3, dX4;
		float dY, dY1, dY2, dY3, dY4;
		int x1, x2, y1, y2;
		for (int j = 0; j < (int)destinationHeight; j ++) {
			//Calculate the position in the calibration map (still with a fractional part)
			float ii = 0.5 * i + destinationWidth / 4;
			float jj = 0.5 * j + destinationHeight / 4;
			calibrationX = 63 * ii / destinationWidth;
			calibrationY = 62 * (1 - jj / destinationHeight); // The y origin is at the bottom
			//Save the fractional part to use as the weight for interpolation
			weightX = calibrationX - truncf(calibrationX);
			weightY = calibrationY - truncf(calibrationY);

			//Get the x,y coordinates of the closest calibration map points to the target pixel
			x1 = calibrationX; //Note truncation to int
			y1 = calibrationY;
			x2 = x1 + 1;
			y2 = y1 + 1;

			//Look up the x and y values for the 4 calibration map points around the target
			dX1 = distortion_buffer[x1 * 2 + y1 * distortionWidth * 2];
			dX2 = distortion_buffer[x2 * 2 + y1 * distortionWidth * 2];
			dX3 = distortion_buffer[x1 * 2 + y2 * distortionWidth * 2];
			dX4 = distortion_buffer[x2 * 2 + y2 * distortionWidth * 2];
			dY1 = distortion_buffer[x1 * 2 + y1 * distortionWidth * 2 + 1];
			dY2 = distortion_buffer[x2 * 2 + y1 * distortionWidth * 2 + 1];
			dY3 = distortion_buffer[x1 * 2 + y2 * distortionWidth * 2 + 1];
			dY4 = distortion_buffer[x2 * 2 + y2 * distortionWidth * 2 + 1];

			//Bilinear interpolation of the looked-up values:
			// X value
			dX = dX1 * (1 - weightX) * (1 - weightY) +
				dX2 * weightX * (1 - weightY) +
				dX3 * (1 - weightX) * weightY +
				dX4 * weightX * weightY;

			// Y value
			dY = dY1 * (1 - weightX) * (1 - weightY) +
				dY2 * weightX * (1 - weightY) +
				dY3 * (1 - weightX) * weightY +
				dY4 * weightX * weightY;

			// Reject points outside the range [0..1]
			if ((dX >= 0) && (dX <= 1) && (dY >= 0) && (dY <= 1)) {
				//Denormalize from [0..1] to [0..width] or [0..height]
				float dx = dX * width;
				float dy = dY * height;
				weightX = dx - truncf(dx);
				weightY = dy - truncf(dy);
				denormalizedX = dx;
				denormalizedY = dy;

				//look up the brightness value for the target pixel
				if (denormalizedX + 1 < width && denormalizedY + 1 < height) {
					destination[i + j * width] = (1 - weightX) * (1 - weightY) * raw[denormalizedX + denormalizedY * width] +
						(weightX) * (1 - weightY) * raw[(denormalizedX + 1) + denormalizedY * width] +
						(1 - weightX) * (weightY) * raw[denormalizedX + (denormalizedY + 1) * width] +
						(weightX) * (weightY) * raw[(denormalizedX + 1) + (denormalizedY + 1) * width];
				}else {
					destination[i + j * width] = 0;
				}
			}
			else {
				destination[i + j * width] = 0;
			}
		}
	}
	
	memcpy(in_raw, destination, 640 * 240 * sizeof(unsigned char));
}

static void OnImage(const LEAP_IMAGE_EVENT *image) {
	Image img;
	solve_distortion((unsigned char*)image->image[0].data, (float*)image->image[0].distortion_matrix, 64);
	int temp = img.display(image->image[0].properties.width, image->image[0].properties.height, image->image[0].data, image->image[1].data);
	if (temp != -1) {
		flag = temp;
	}
	if (flag == 2) {
		if (frames.size() > 0) {
			writeFrames(string(name) + string("_") + to_string(cnt) + string(".txt"));
			cnt++;
			frames.clear();
		}
		flag = 1;
		if (cnt > 9) {
			flag = 4;
		}
		std::cout << "Next " << cnt << std::endl;
		printMsg();
	}
	else if (flag == 3) {
		frames.clear();
		flag = 1;
		std::cout << "Redo " << cnt << std::endl;
		printMsg();
	}
	else if (flag == 4) {
		frames.clear();
		flag = -1;
		std::cout << "Quit " << cnt << std::endl;
	}
	// std::cout << "flag: " << flag << std::endl;
	/*
	printf("Image %lli  => Left: %d x %d (bpp=%d), Right: %d x %d (bpp=%d)\n",
		(long long int)image->info.frame_id,
		image->image[0].properties.width, image->image[0].properties.height, image->image[0].properties.bpp * 8,
		image->image[1].properties.width, image->image[1].properties.height, image->image[1].properties.bpp * 8);
		*/
	// images.push_back(img);
}

static void OnLogMessage(const eLeapLogSeverity severity, const int64_t timestamp,
	const char* message) {
	const char* severity_str;
	switch (severity) {
	case eLeapLogSeverity_Critical:
		severity_str = "Critical";
		break;
	case eLeapLogSeverity_Warning:
		severity_str = "Warning";
		break;
	case eLeapLogSeverity_Information:
		severity_str = "Info";
		break;
	default:
		severity_str = "";
		break;
	}
	printf("[%s][%lli] %s\n", severity_str, (long long int)timestamp, message);
}

static void* allocate(uint32_t size, eLeapAllocatorType typeHint, void* state) {
	void* ptr = malloc(size);
	return ptr;
}

static void deallocate(void* ptr, void* state) {
	if (!ptr)
		return;
	free(ptr);
}

void OnPointMappingChange(const LEAP_POINT_MAPPING_CHANGE_EVENT *change) {
	if (!connectionHandle)
		return;

	uint64_t size = 0;
	if (LeapGetPointMappingSize(*connectionHandle, &size) != eLeapRS_Success || !size)
		return;

	LEAP_POINT_MAPPING* pointMapping = (LEAP_POINT_MAPPING*)malloc(size);
	if (!pointMapping)
		return;

	if (LeapGetPointMapping(*connectionHandle, pointMapping, &size) == eLeapRS_Success &&
		pointMapping->nPoints > 0) {
		printf("Managing %u points as of frame %lld at %lld\n", pointMapping->nPoints, (long long int)pointMapping->frame_id, (long long int)pointMapping->timestamp);
	}
	free(pointMapping);
}

void OnHeadPose(const LEAP_HEAD_POSE_EVENT *event) {
	printf("Head pose:\n");
	printf("    Head position (%f, %f, %f).\n",
		event->head_position.x,
		event->head_position.y,
		event->head_position.z);
	printf("    Head orientation (%f, %f, %f, %f).\n",
		event->head_orientation.w,
		event->head_orientation.x,
		event->head_orientation.y,
		event->head_orientation.z);
}

void writeFrames(string fileName) {
	ofstream fout(fileName);
	for (vector<Frame>::iterator iter = frames.begin(); iter != frames.end(); iter++) {
		fout << iter->timestamp << " "
			<< iter->grab_angle << " "
			<< iter->grab_strength << " "
			<< iter->palm.realPos.at<double>(0, 0) << " "
			<< iter->palm.realPos.at<double>(1, 0) << " "
			<< iter->palm.realPos.at<double>(2, 0) << " "
			<< iter->palm.realNormal.at<double>(0, 0) << " "
			<< iter->palm.realNormal.at<double>(1, 0) << " "
			<< iter->palm.realNormal.at<double>(2, 0) << " "
			<< iter->palm.realDirection.at<double>(0, 0) << " "
			<< iter->palm.realDirection.at<double>(1, 0) << " "
			<< iter->palm.realDirection.at<double>(2, 0) << " ";
		for (int i = 0; i < 5; i++) {
			fout << iter->fingers[i].id << " "
				<< iter->fingers[i].extended << " ";
			fout << iter->fingers[i].bones[0].realPrev.at<double>(0, 0) << " "
				<< iter->fingers[i].bones[0].realPrev.at<double>(1, 0) << " "
				<< iter->fingers[i].bones[0].realPrev.at<double>(2, 0) << " "
				<< iter->fingers[i].bones[0].realNext.at<double>(0, 0) << " "
				<< iter->fingers[i].bones[0].realNext.at<double>(1, 0) << " "
				<< iter->fingers[i].bones[0].realNext.at<double>(2, 0) << " ";
			for (int j = 1; j < 4; j++) {
				fout << iter->fingers[i].bones[j].realNext.at<double>(0, 0) << " "
					<< iter->fingers[i].bones[j].realNext.at<double>(1, 0) << " "
					<< iter->fingers[i].bones[j].realNext.at<double>(2, 0) << " ";
			}
		}
		fout << iter->rv(0) << " " << iter->rv(1) << " " << iter->rv(2) << " "
			<< iter->tv(0) << " " << iter->tv(1) << " " << iter->tv(2);
		fout << endl;
	}
	fout.close();
	std::cout << "Frames written" << std::endl;
}

int main(int argc, char** argv) {
	//Set callback function pointers
	ConnectionCallbacks.on_connection = &OnConnect;
	ConnectionCallbacks.on_device_found = &OnDevice;
	ConnectionCallbacks.on_frame = &OnFrame;
	ConnectionCallbacks.on_image = &OnImage;
	ConnectionCallbacks.on_point_mapping_change = &OnPointMappingChange;
	ConnectionCallbacks.on_log_message = &OnLogMessage;
	ConnectionCallbacks.on_head_pose = &OnHeadPose;

	string file = "config.txt";
	ifstream infile;
	infile.open(file.data());
	assert(infile.is_open());

	getline(infile, name);
	infile.close();	

	start = clock();

	connectionHandle = OpenConnection();
	{
		LEAP_ALLOCATOR allocator = { allocate, deallocate, NULL };
		LeapSetAllocator(*connectionHandle, &allocator);
	}
	LeapSetPolicyFlags(*connectionHandle, eLeapPolicyFlag_Images | eLeapPolicyFlag_MapPoints | eLeapPolicyFlag_OptimizeHMD, 0);

	printf("Press Enter to exit program.\n");
	getchar();

	// DestroyConnection();

	return 0;
}
//End-of-Sample.c
