#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

#include "MDetector.hpp"

using namespace std;
using namespace cv;

#define DEBUG_DISPLAY 0

MDetector::MDetector(int nthre, int bthre, int margin, Size grid_size) :
	m_nthre(nthre), m_bthre(bthre), m_gridSize(grid_size)
{
	m_termcrit = TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.3);
	m_orb = ORB::create();
	m_kernel = getStructuringElement(MORPH_ELLIPSE, Size(margin, margin));
}

void MDetector::init(const Mat & frame)
{
	cvtColor(frame, m_preGray, CV_BGR2GRAY);
	for (size_t i = m_gridSize.width / 2; i < frame.cols - m_gridSize.width / 2; i += m_gridSize.width)
		for (size_t j = m_gridSize.height / 2; j < frame.rows - m_gridSize.height / 2; j += m_gridSize.height)
			m_gridPoints.emplace_back(i, j);

	// init frame corners
	m_corners.emplace_back(0, 0);
	m_corners.emplace_back(frame.cols, 0);
	m_corners.emplace_back(frame.cols, frame.rows);
	m_corners.emplace_back(0, frame.rows);
}

vector<cv::Rect> MDetector::run(const Mat & frame)
{
	Mat gray;
	cvtColor(frame, gray, CV_BGR2GRAY);
	vector<KeyPoint> kpoints;
	m_orb->detect(gray, kpoints);
	vector<Point2f> orb_points;
	for (auto & kpt : kpoints)
		orb_points.push_back(kpt.pt);
	
	vector<Rect> rois = boundingBoxes(orb_points);

	if (rois.size() > m_nthre)
		rois = gridFlowSubtaction(gray, orb_points);
	else
		rois = directSubtaction(gray, orb_points);

	m_preGray = gray.clone();
	return rois;
}

std::vector<cv::Rect> MDetector::directSubtaction(const Mat & gray, const vector<Point2f>& points)
{
	Mat mask;
	subtract(gray, m_preGray, mask);
	threshold(mask, mask, m_bthre, 255, THRESH_BINARY);
	dilate(mask, mask, m_kernel);

	vector<Point2f> alive_points;
	for (auto & pt : points) {
		if (mask.at<uchar>(pt) > 0)
			alive_points.push_back(pt);
	}

	return boundingBoxes(alive_points);
}

std::vector<cv::Rect> MDetector::gridFlowSubtaction(const Mat & gray, const vector<Point2f> & points)
{
	cornerSubPix(m_preGray, m_gridPoints, Size(5, 5), Size(-1, -1), m_termcrit);
	vector<uchar> flow_status;
	vector<float> flow_err;
	vector<Point2f> currPoints, preTemp, currTemp;
	calcOpticalFlowPyrLK(m_preGray, gray, m_gridPoints, currPoints, flow_status, flow_err);
	for (size_t i = 0; i < m_gridPoints.size(); i++) {
		if (flow_status[i]) {
			preTemp.push_back(m_gridPoints[i]);
			currTemp.push_back(currPoints[i]);
		}
	}
#if DEBUG_DISPLAY
	drawTrace(gray, preTemp, currTemp);
#endif
	Mat hMat = findHomography(preTemp, currTemp, RANSAC);
	Mat mask;
	warpPerspective(m_preGray, mask, hMat, m_preGray.size(), INTER_LINEAR, BORDER_CONSTANT, 0);
	maskBorderLine(mask, hMat);
	subtract(gray, mask, mask, mask);
	threshold(mask, mask, m_bthre, 255, THRESH_BINARY);
	dilate(mask, mask, m_kernel);
	vector<Point2f> alive_points;
	for (auto & pt : points) {
		if (mask.at<uchar>(pt) > 0)
			alive_points.push_back(pt);
	}

	return boundingBoxes(alive_points);
}

std::vector<cv::Rect> MDetector::boundingBoxes(const vector<Point2f> & points)
{
	Mat mask = Mat::zeros(m_preGray.size(), CV_8UC1);
	drawPoints(mask, points);
	vector<vector<Point>> contours;
	findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	vector<Rect> rois;
	for (auto & con : contours)
		rois.push_back(boundingRect(con));

	return rois;
}

void MDetector::drawPoints(Mat & mask, const vector<Point2f> & points)
{
	CV_Assert(!mask.empty());
	for (auto & pt : points)
		circle(mask, pt, 16, 255, -1);
}

void MDetector::drawTrace(const Mat & image, const vector<Point2f>& pre, const vector<Point2f>& curr)
{
	static bool created = false;
	static String window_name = "trace";
	if (!created) {
		namedWindow(window_name, WINDOW_AUTOSIZE);
		created = true;
	}
	Mat vis = image.clone();
	for (size_t i = 0; i < pre.size(); i++)
		line(vis, pre[i], curr[i], Scalar(255, 0, 0));
	imshow(window_name, vis);
}

void MDetector::maskBorderLine(Mat & mask, const Mat & hMat, int thickness)
{
	vector<Point2f> aligned_corners(4);
	perspectiveTransform(m_corners, aligned_corners, hMat);
	line(mask, aligned_corners[0], aligned_corners[1], 0, thickness);
	line(mask, aligned_corners[1], aligned_corners[2], 0, thickness);
	line(mask, aligned_corners[2], aligned_corners[3], 0, thickness);
	line(mask, aligned_corners[3], aligned_corners[0], 0, thickness);
}
