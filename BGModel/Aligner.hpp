#pragma once
#include <vector>
#include <opencv2/features2d.hpp>
#include "cv.h"

enum FeatureType { FT_GOOD, FT_ORB, FT_GRID };
enum HomographyMethod { HM_MATCH, HM_FLOW };

class Aligner {
public:
	Aligner(FeatureType ft = FT_ORB, HomographyMethod hm = HM_FLOW);
	void init(const cv::Mat & frame);
	cv::Mat run(const cv::Mat & frame);
private:
	FeatureType m_ftype;
	HomographyMethod m_hMethod;
	cv::Mat m_preGray, m_currGray;
	cv::Mat m_HMat;
	cv::TermCriteria m_termCriteria;
	cv::Ptr<cv::ORB> m_orb;
	cv::Ptr<cv::DescriptorMatcher> m_matcher;
	std::vector<cv::KeyPoint> m_preKPoints, m_currKPoints;
	std::vector<cv::Point2f> m_prePoints, m_currPoints;
	std::vector<cv::Point2f> m_corners;
	cv::Mat m_preDescriptors, m_currDescriptors;
	void drawTrace(const cv::Mat & image, const std::vector<cv::Point2f> & pre, const std::vector<cv::Point2f> & curr);
	void getGridFeatures(const cv::Mat & image, std::vector<cv::Point2f> & points, const cv::Size grid_size = cv::Size(64, 48));
	void maskBorderLine(cv::Mat & mask, int thickness = 4);
};
