/************************************************************************/
/* namespace cv                                                        */
/************************************************************************/
#include "Opencv.hpp"

class Draw
{
public:
	virtual void drawRotateRect(Mat& src, RotatedRect& rr)=0;
    virtual void drawConvexHull(Mat& src, PointSet& hull, Scalar color)=0;

    virtual void drawBlack(SegmentSet& blackRegions, Mat& disp, Vec3b& color)=0;
    virtual void draw(SegmentSet& segment, Mat& disp, vector<Vec3b>& colors)=0;
    virtual void drawBoundBox(SegmentSet& segment, vector<double>& distance, Mat& color, Mat& depth, string categoryName)=0;
    virtual void drawRegions(SegmentSet& segment, Mat& color, Mat& depth, Mat& disp)=0;

    virtual void drawSobel(Mat& depth)=0;
};

