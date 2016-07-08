#ifndef SEGMENTATION_HPP
#define SEGMENTATION_HPP

#endif // SEGMENTATION_HPP

#include "Opencv.hpp"
#include "MyDraw.hpp"

//#define imshow(_W_, _M_) imwrite(_W_+std::string(".png"), _M_)


class Segmentation : public Draw
{
public:
	Segmentation(int width, int height, unsigned topk, short t);
	Segmentation(Size sz, unsigned topk, short t);

	void Segment(Mat& depth, Mat& color);
	void DFS(Mat& depth, Mat& visit, Point cur, short& threshold, vector<Point>& v);
	void NonRecursive(Mat& depth, Mat& visit, Point& cur, PointSet& pSet);
	void completeDepth(Mat& depth);
	void clear();
private:
	void regionMerge(Mat& depth, SegmentSet& segment, SegmentSet& blackRegions, unsigned topk, double minSim);
	inline void calculateConvexHulls();
	inline void calculateBoundBoxex();
	inline Rect hullBoundBox(PointSet& hull);

	inline bool isRegionInsideHull(PointSet& pSet, PointSet& hull, double minSim);
	inline bool isRegionInsideHull(PointSet& pSet, PointSet& hull, PointSet& seg, double minSim);

	void randColor();

	//Extend Draw
	virtual void drawRotateRect(Mat& src, RotatedRect& rr);
	virtual void drawConvexHull(Mat& src, PointSet& hull, Scalar color);

	virtual void drawBlack(SegmentSet& blackRegions, Mat& disp, Vec3b& color);
	virtual void draw(SegmentSet& segment, Mat& disp, vector<Vec3b>& colors);
	virtual void drawBoundBox(SegmentSet& segment, vector<double>& distance, Mat& color, Mat& depth, string categoryName = "bottle");
	virtual void drawRegions(SegmentSet& segment, Mat& color, Mat& depth, Mat& disp);

	virtual void drawSobel(Mat& depth);



public:
	SegmentSet mainRegions_;
	SegmentSet blackRegions_;
	vector<PointSet> convexHulls_;
	vector<Rect> boundBoxes_;
	vector<double> distance_;

private:
	static const vector<Point> _DIRECTIONS_;
	static const vector<Point> _DIR_;
	vector<Vec3b> colors_;
	Rect RANGE_;
	short threshold_;
	unsigned topk_;
};
