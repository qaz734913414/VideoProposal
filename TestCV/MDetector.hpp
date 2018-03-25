#pragma once
#include <vector>
#include <opencv2/features2d.hpp>
#include "cv.h"

// motion detection
// usr orb feature with grid optical flow.

class MDetector {
public:
	MDetector(int nthre = 5, int bthre = 30, int margin = 5, cv::Size grid_size = cv::Size(32, 24));
	void init(const cv::Mat & frame);
	std::vector<cv::Rect> run(const cv::Mat & frame);

private:
	cv::Mat m_preGray; // the pre gray frame;
	cv::TermCriteria m_termcrit;	// stop condition for iteration
	cv::Ptr<cv::ORB> m_orb;	// ORB feature detector
	std::vector<cv::Point2f> m_gridPoints; // grid feature points for grid optical flow;
	std::vector<cv::Point2f> m_corners;
	cv::Size m_gridSize;
	int m_nthre;	// used to choose motion background subtraction.
	int m_bthre;	// used for background substraction threshold
	cv::Mat m_kernel; // expand margin for background subtraction

	std::vector<cv::Rect> directSubtaction(const cv::Mat & gray, const std::vector<cv::Point2f> & points);
	std::vector<cv::Rect> gridFlowSubtaction(const cv::Mat & gray, const std::vector<cv::Point2f> & points);
	std::vector<cv::Rect> boundingBoxes(const std::vector<cv::Point2f> & points);
	void drawPoints(cv::Mat & mask, const std::vector<cv::Point2f> & points);
	void drawTrace(const cv::Mat & image, const std::vector<cv::Point2f> & pre, const std::vector<cv::Point2f> & curr);
	void maskBorderLine(cv::Mat & mask, const cv::Mat & hMat, int thickness = 4);
};