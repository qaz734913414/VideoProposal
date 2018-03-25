#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#include "Aligner.hpp"

using namespace std;
using namespace cv;

Aligner::Aligner(FeatureType ft, HomographyMethod hm) :
	m_ftype(ft), m_hMethod(hm)
{
	m_termCriteria = TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.3);
	m_orb = ORB::create();
	m_matcher = DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE_L1);
}

void Aligner::init(const Mat & frame)
{
	cvtColor(frame, m_preGray, CV_BGR2GRAY);
	if (m_hMethod == HM_FLOW) {
		if (m_ftype == FT_ORB) {
			m_orb->detect(m_preGray, m_preKPoints);
			for (auto & kp : m_preKPoints)
				m_prePoints.push_back(kp.pt);
		}
		else if (m_ftype == FT_GOOD)
			goodFeaturesToTrack(m_preGray, m_prePoints, 500, 0.01, 30);
		else // FT_GRID
			getGridFeatures(m_preGray, m_prePoints);
		cornerSubPix(m_preGray, m_prePoints, Size(5, 5), Size(-1, -1), m_termCriteria);
	}
	else { // HM_MATCH
		if (m_ftype == FT_ORB) {
			m_orb->detect(m_preGray, m_preKPoints);
			m_orb->compute(m_preGray, m_preKPoints, m_preDescriptors);
		}
		else // FT_GOOD or FT_GRID
			throw runtime_error("Not implemented!");
	}

	// init frame corners
	m_corners.emplace_back(0, 0);
	m_corners.emplace_back(frame.cols, 0);
	m_corners.emplace_back(frame.cols, frame.rows);
	m_corners.emplace_back(0, frame.rows);
}

Mat Aligner::run(const Mat & frame)
{
	cvtColor(frame, m_currGray, CV_BGR2GRAY);
	if (m_hMethod == HM_FLOW) {
		vector<uchar> flow_status;
		vector<float> flow_err;
		vector<Point2f> preTemp, currTemp;
		calcOpticalFlowPyrLK(m_preGray, m_currGray, m_prePoints, m_currPoints, flow_status, flow_err);
		for (size_t i = 0; i < m_prePoints.size(); i++) {
			if (flow_status[i]) {
				preTemp.push_back(m_prePoints[i]);
				currTemp.push_back(m_currPoints[i]);
			}
		}
		drawTrace(frame, preTemp, currTemp);
		m_HMat = findHomography(preTemp, currTemp, RANSAC);

		if (m_ftype == FT_ORB) {
			m_orb->detect(m_currGray, m_preKPoints);
			for (auto & kp : m_preKPoints)
				m_prePoints.push_back(kp.pt);
		}
		else if (m_ftype == FT_GOOD)
			goodFeaturesToTrack(m_currGray, m_prePoints, 500, 0.01, 30);
		else // FT_GRID
			; // No need to update grid feature points
	}
	else { // HM_MATCH
		if (m_ftype == FT_ORB) {
			m_orb->detect(m_currGray, m_currKPoints);
			m_orb->compute(m_currGray, m_currKPoints, m_currDescriptors);
			vector<DMatch> matchs, matches12, matches21;
			// symmetric match
			m_matcher->match(m_preDescriptors, m_currDescriptors, matches12);
			m_matcher->match(m_currDescriptors, m_preDescriptors, matches21);
			for (size_t i = 0; i < matches12.size(); i++) {
				DMatch _forward = matches12[i];
				DMatch _backward = matches12[_forward.trainIdx];
				// a -> b && b -> a
				if (_forward.queryIdx == _backward.trainIdx)
					matchs.push_back(_forward);
			}
			m_prePoints.clear();
			m_currPoints.clear();
			for (auto & match : matchs) {
				m_prePoints.push_back(m_preKPoints[match.queryIdx].pt);
				m_currPoints.push_back(m_currKPoints[match.trainIdx].pt);
			}
			drawTrace(frame, m_prePoints, m_currPoints);
			cout << m_prePoints.size() << " " << m_currPoints.size() << endl;
			m_HMat = findHomography(m_prePoints, m_currPoints, RANSAC);

			m_preKPoints = m_currKPoints;
			m_preDescriptors = m_currDescriptors;
		}
		else // FT_GOOD or FT_GRID
			throw runtime_error("Not implemented!");
	}

	Mat mask;
	warpPerspective(m_preGray, mask, m_HMat, m_preGray.size(), INTER_LINEAR, BORDER_CONSTANT, 0);
	maskBorderLine(mask);
	subtract(m_currGray, mask, mask, mask);
	threshold(mask, mask, 30, 255, THRESH_BINARY);

	m_preGray = m_currGray.clone();
	return mask;
}

void Aligner::drawTrace(const Mat & image, const vector<Point2f> & pre, const vector<Point2f> & curr)
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

void Aligner::getGridFeatures(const Mat & image, vector<Point2f> & points, const Size grid_size)
{
	for (size_t i = grid_size.width / 2; i < image.cols - grid_size.width / 2; i += grid_size.width)
		for (size_t j = grid_size.height / 2; j < image.rows - grid_size.height / 2; j += grid_size.height)
			points.emplace_back(i, j);
}

void  Aligner::maskBorderLine(Mat & mask, int thickness)
{
	vector<Point2f> aligned_corners(4);
	perspectiveTransform(m_corners, aligned_corners, m_HMat);
	line(mask, aligned_corners[0], aligned_corners[1], 0, thickness);
	line(mask, aligned_corners[1], aligned_corners[2], 0, thickness);
	line(mask, aligned_corners[2], aligned_corners[3], 0, thickness);
	line(mask, aligned_corners[3], aligned_corners[0], 0, thickness);
}