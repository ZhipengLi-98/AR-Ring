#undef __cplusplus

#include <stdio.h>
#include <stdlib.h>
#include "LeapC.h"
#include "ExampleConnection.h"

#include "frame.h"
#include <vector>
#include <ctime>
#include <fstream>

vector<Frame> frames;
vector<Image> images;

Image imageInst;

int start;

static LEAP_CONNECTION* connectionHandle;

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

	// 默认下应该只有一只手
	// assert frame->nHands == 1
	for (uint32_t h = 0; h < frame->nHands; h++) {
		Frame next;
		LEAP_HAND* hand = &frame->pHands[h];
		next.setData(hand->grab_angle, hand->grab_strength, hand->palm, hand->digits);
		next.convert();
		int now = clock();
		next.timestamp = (double)(now);
		/*
		printf("    Hand id %i is a %s hand with position (%f, %f, %f).\n",
			hand->id,
			(hand->type == eLeapHandType_Left ? "left" : "right"),
			hand->palm.position.x,
			hand->palm.position.y,
			hand->palm.position.z);
			*/

		frames.push_back(next);
	}
}

static void OnImage(const LEAP_IMAGE_EVENT *image) {
	Image img;
	int now = clock();
	img.timestamp = (double)now;
	img.display(image->image[0].properties.width, image->image[0].properties.height, image->image[0].data, image->image[1].data);
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

void writeFrames(char* fileName) {
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
		fout << endl;
	}
	fout.close();
	std::cout << "Frames written" << std::endl;
}

void writeImages(char* fileName) {
	std::cout << "Images written" << std::endl;
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

	start = clock();

	connectionHandle = OpenConnection();
	{
		LEAP_ALLOCATOR allocator = { allocate, deallocate, NULL };
		LeapSetAllocator(*connectionHandle, &allocator);
	}
	LeapSetPolicyFlags(*connectionHandle, eLeapPolicyFlag_Images | eLeapPolicyFlag_MapPoints | eLeapPolicyFlag_OptimizeHMD, 0);

	printf("Press Enter to exit program.\n");
	getchar();
	std::cout << "Please Enter File Name" << std::endl;
	char namef[105], namei[105];
	std::cin >> namef;
	
	writeFrames(namef);
	// writeImages(namei);

	// DestroyConnection();

	return 0;
}
//End-of-Sample.c
