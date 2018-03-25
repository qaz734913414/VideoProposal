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
