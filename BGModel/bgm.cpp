#include <iostream>
#include <list>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include "Aligner.hpp"

//#pragma comment(lib, CV_LIB(videoio))
//#pragma comment(lib, CV_LIB(highgui))

using namespace std;
using namespace cv;


int main()
{
	setUseOptimized(true);
	setNumThreads(8);
	string video = "F:/SmallObjectDetection/video/small-3.mp4";
	// string video = "F:/UAV-videos/Small/small-4.mp4";
	// string video = "F:/UAV-videos/Small/small-2.mp4";
	// string video = "F:/UAV-videos/Big/big-1.mp4";
	// string video = "F:/UAV-videos/NoSky/nosky-2.mp4";
	// string video = "F:/UAV-videos/uav.MP4";
	// string video = "F:/UAV-videos/testx.mp4";
	// string video = "F:/fengtao/QQ ”∆µ20180201103440.mp4";
	// namedWindow("image", WINDOW_AUTOSIZE);
	namedWindow("mask", WINDOW_AUTOSIZE);

	VideoCapture capture(video);
	if (!capture.isOpened()) {
		cerr << "Cannot read Video from " << video << endl;
		return -1;
	}

	Mat frame;
	capture >> frame;
	if (frame.empty()) {
		cerr << "Fail to read in video." << endl;
		return -1;
	}

	// Aligner aligner(FT_GOOD, HM_FLOW);
	// Aligner aligner(FT_ORB, HM_FLOW);
	// Aligner aligner(FT_ORB, HM_MATCH);
	Aligner aligner(FT_GRID, HM_FLOW);
	capture >> frame;
	aligner.init(frame);

	double t;
	for (int num_frame = 1; ; num_frame++)
	{
		//for (size_t i = 0; i < 5; i++)
		//	capture >> frame;
		capture >> frame;
		if (frame.empty())
			break;

		t = (double)getTickCount();
		Mat mask = aligner.run(frame);
		// imshow("image", frame);
		imshow("mask", mask);

		t = ((double)getTickCount() - t) / getTickFrequency() * 1000;

		int c = waitKey(50);
		if (c == 'q' || c == 'Q' || (c & 255) == 27)
			break;

	}

	return 0;
}
