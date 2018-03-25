#include <iostream>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "MDetector.hpp"
#include <io.h>
#include <direct.h>
using namespace std;
using namespace cv;

const String about = "\n"
	"Extracting region proposals from video with OpenCV " + String(CV_VERSION) + "\n";

const String keys =
	"{help h usage ? |     | print this message }"
	"{@video         |  F:/SmallObjectDetection/video/big-1.mp4   | path to a video file}"
	"{@output        |  F:/SmallObjectDetection/video/big-1   | path to output directory}"
	"{n nthre        | 5   | used to choose motion background subtraction}"
	"{b bthre        | 30  | used for threshold background subtraction mask}"
	"{m margin       | 5   | expand mask for feature filtering}"
	"{w width        | 32  | width of grid for optical flow motion}"
	"{h height       | 24  | height of grid for optical flow motion}"
	;

int main(int argc, char** argv)
{
	setUseOptimized(true);
	setNumThreads(8);

	CommandLineParser parser(argc, argv, keys);
	parser.about(about);
	if (parser.has("help")) {
		parser.printMessage();
		return 0;
	}

	String video = parser.get<String>(0);
	String output = parser.get<String>(1);
	int nthre = parser.get<int>("nthre");
	int bthre = parser.get<int>("bthre");
	int margin = parser.get<int>("margin");
	int grid_w = parser.get<int>("width");
	int grid_h = parser.get<int>("height");

	if (!parser.check()) {
		parser.printErrors();
		return -1;
	}

	VideoCapture capture(video);
	if (!capture.isOpened()) {
		std::cerr << "Cannot read Video from " << video << std::endl;
		return -1;
	}

	MDetector detector(nthre, bthre, margin, Size(grid_w, grid_h));

	Mat frame;
	capture >> frame;
	if (frame.empty()) {
		cerr << "Fail to read in video." << endl;
		return -1;
	}
	detector.init(frame);

	if (_access(output.c_str(), NULL) == -1) {
		_mkdir(output.c_str());
	}

	double t;
	for (int frame_id = 1; ; frame_id++)
	{
		capture >> frame;
		if (frame.empty())
			break;

		t = (double)getTickCount();
		vector<Rect> rois = detector.run(frame);
		t = ((double)getTickCount() - t) / getTickFrequency() * 1000;
		for (size_t i = 0; i < rois.size(); i++) {
			imwrite(output + "/" + to_string(frame_id) + "_" + to_string(i) + ".png", frame(rois[i]));
		}
		cout << "\r frame " << frame_id << ": " << t << " ms, " << rois.size() << " proposals";
		cout.flush();
	}

	return 0;
}

//#include <iostream>
//#include <opencv2/videoio.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/imgproc.hpp>
//#include "MDetector.hpp"
//
//using namespace std;
//using namespace cv;
//
//int main()
//{
//	setUseOptimized(true);
//	setNumThreads(8);
//	// string video = "F:/SmallObjectDetection/video/small-3.mp4";
//	// string video = "F:/UAV-videos/Small/small-4.mp4";
//	// string video = "F:/UAV-videos/Small/small-2.mp4";
//	// string video = "F:/UAV-videos/Big/big-1.mp4";
//	// string video = "F:/UAV-videos/NoSky/nosky-2.mp4";
//	// string video = "F:/UAV-videos/uav.MP4";
//	// string video = "F:/UAV-videos/testx.mp4";
//	string video = "F:/fengtao/QQ ”∆µ20180201103440.mp4";
//	namedWindow("Bounding Box", WINDOW_AUTOSIZE);
//
//	VideoCapture capture(video);
//	if (!capture.isOpened()) {
//		cerr << "Cannot read Video from " << video << endl;
//		return -1;
//	}
//
//	Mat frame;
//	capture >> frame;
//	if (frame.empty()) {
//		cerr << "Fail to read in video." << endl;
//		return -1;
//	}
//
//	MDetector detector;
//	detector.init(frame);
//	double t;
//	for (int num_frame = 1; ; num_frame++)
//	{
//		capture >> frame;
//		if (frame.empty())
//			break;
//
//		t = (double)getTickCount();
//		vector<Rect> rois = detector.run(frame);
//		t = ((double)getTickCount() - t) / getTickFrequency() * 1000;
//		for (auto & roi : rois)
//			rectangle(frame, roi, Scalar(255, 0, 0), 2);
//		imshow("Bounding Box", frame);
//
//		int c = waitKey(1);
//		if (c == 'q' || c == 'Q' || (c & 255) == 27)
//			break;
//	}
//
//	return 0;
//}