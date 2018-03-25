#pragma once
// pragma warning(disable: 4996)
//#define OPENCV_TRAITS_ENABLE_DEPRECATED
#include<opencv2/core.hpp>

#define CV_VERSION_ID CVAUX_STR(CV_MAJOR_VERSION) \
	CVAUX_STR(CV_MINOR_VERSION) CVAUX_STR(CV_SUBMINOR_VERSION)
#ifdef _DEBUG
#define CV_LIB(name) "opencv_" #name CV_VERSION_ID "d"
#else
#define CV_LIB(name) "opencv_" #name CV_VERSION_ID
#endif // _DEBUG

//#pragma comment(lib, CV_LIB(core))
#pragma comment(lib, CV_LIB(world))

//#include "cv.h"
//#include <opencv2/opencv.hpp>
//#include <vector>
//#include <iostream>
//#include <sstream>
//using namespace std;
//using namespace cv;
//
//#pragma comment(lib, CV_LIB(core))
//#pragma comment(lib, CV_LIB(videoio))
//#pragma comment(lib, CV_LIB(highgui))
//#pragma comment(lib, CV_LIB(imgproc))
//#pragma comment(lib, CV_LIB(features2d))
//
//void mDrawKeypoints(InputOutputArray image, const std::vector<KeyPoint>& keypoints,
//	int radius, const Scalar& color, int thickness)
//{
//	CV_Assert(!image.empty());
//	for (const auto& keypoint : keypoints)
//		circle(image, keypoint.pt, radius, color, thickness);
//}
//
//void mDrawKeypoints(InputArray image, InputOutputArray outImage,
//	std::vector<KeyPoint>& keypoints, int radius, const Scalar& color,
//	int thickness, InputArray mask = noArray())
//{
//	CV_Assert(!image.empty());
//	if (outImage.empty())
//		image.copyTo(outImage);
//	if (mask.empty()) {
//		for (const auto& keypoint : keypoints)
//			circle(outImage, keypoint.pt, radius, color, thickness);
//	}
//	else {
//		Mat _mask = mask.getMat();
//		for (const auto& keypoint : keypoints) {
//			if (_mask.at<uchar>(keypoint.pt))
//				circle(outImage, keypoint.pt, radius, color, thickness);
//		}
//	}
//}
//
//int main()
//{
//	setUseOptimized(true);
//	setNumThreads(8);
//	string video = "F:/UAV-videos/Small/small-4.mp4";
//	// string video = "F:/UAV-videos/Small/small-2.mp4";
//	// string video = "F:/UAV-videos/Big/big-1.mp4";
//	// string video = "F:/UAV-videos/NoSky/nosky-2.mp4";
//	// string video = "F:/UAV-videos/uav.MP4";
//	// string video = "F:/UAV-videos/testx.mp4";
//	// string video = "F:/fengtao/QQ ”∆µ20180201103440.mp4";
//	//namedWindow("Fast", WINDOW_AUTOSIZE);
//	namedWindow("ORB", WINDOW_AUTOSIZE);
//	namedWindow("mask", WINDOW_AUTOSIZE);
//	VideoCapture capture(video);
//	if (!capture.isOpened()) {
//		std::cerr << "Cannot read Video from " << video << std::endl;
//		return -1;
//	}
//
//	int width = capture.get(CAP_PROP_FRAME_WIDTH);
//	int height = capture.get(CAP_PROP_FRAME_HEIGHT);
//	Mat frame, mask, pre_mask;
//	Mat blackboard = Mat::zeros(height, width, CV_8UC1);
//	vector<KeyPoint> keypoints;
//	//Ptr<ORB> orbD = ORB::create();
//	Ptr<ORB> orbD = ORB::create(100, 1.2F, 8, 31, 0, 2, 0, 31, 20);
//	stringstream ss;
//	double t;
//	for (int num_frame = 1; ; num_frame++)
//	{
//		capture >> frame;
//		if (frame.empty())
//			break;
//
//		t = getTickCount();
//		orbD->detect(frame, keypoints);
//		if (!pre_mask.empty()) {
//			mDrawKeypoints(blackboard, mask, keypoints, 16, 255, -1);
//			vector<vector<Point>> contours;
//			findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//			for (const auto& contour : contours)
//				rectangle(frame, boundingRect(contour), Scalar(255, 0, 0), 4);
//			t = ((double)getTickCount() - t) / getTickFrequency() * 1000;
//			ss.str("");
//			ss << num_frame << ": " << keypoints.size() << " " << t << "ms";
//			putText(frame, ss.str(), Point(10, 50),
//				FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0), 2);
//			imshow("ORB", frame);
//			imshow("mask", mask);
//
//			pre_mask = mask;
//			mask = Mat();
//		}
//		else
//			mDrawKeypoints(blackboard, pre_mask, keypoints, 16, 255, -1);
//
//		int c = waitKey(1);
//		if (c == 'q' || c == 'Q' || (c & 255) == 27)
//			break;
//
//	}
//
//	return 0;
//}
