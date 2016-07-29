#pragma once
/************************************************************************/
/* namespace std                                                        */
/************************************************************************/
#include "Common.hpp"

/************************************************************************/
/* namespace cv                                                         */
/************************************************************************/
#include <opencv2/opencv.hpp>
using cv::Mat;
using cv::Size;
using cv::Point;
using cv::Point2f;
using cv::Scalar;
using cv::Vec3b;
using cv::Rect;
using cv::RotatedRect;
using cv::Exception;

using cv::waitKey;
using cv::imshow;
using cv::imread;
using cv::imwrite;
using cv::line;
using cv::rectangle;

/************************************************************************/
/* Typedef                                                              */
/************************************************************************/
typedef std::vector<cv::Point> PointSet;
typedef std::vector<PointSet> SegmentSet;
//typedef std::vector<PointSet> ConvexHullSet;


// replace imshow to imwrite
namespace MYCUSTOM{
	static int mynumber = 0;
}

//#define imshow(_W_, _M_) imwrite(to_string(MYCUSTOM::mynumber)+_W_+std::string(".png"), _M_)