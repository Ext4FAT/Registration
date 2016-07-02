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
typedef std::vector< std::vector<cv::Point> > SegmentSet;

