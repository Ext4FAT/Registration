/************************************************************************/
/* namespace cv                                                         */
/************************************************************************/
#include "Opencv.hpp"
#include "MyDraw.hpp"

/************************************************************************/
/* Improved region-growing segmentation                                 */
/************************************************************************/
class Segmentation : public Draw
{
public:
	/** 
    * @brief Constructor with parameters 
	*/
	Segmentation(int width, int height, unsigned topk, short t);
	Segmentation(Size sz, unsigned topk, short t);
	/**
	* @brief DFS: depth first search with recurrsive methold, but easy to stack overflow
	* @param depth, color	must be the same resolution
	*/
	void Segment(Mat& depth, Mat& color);
	/**
	* @brief DFS: depth first search with recurrsive methold, but easy to stack overflow
	* @param depth         depth data
	* @param visit         marked visited point
	* @param cur           current point position
	* @param threshold     the max value between extend point
	* @param v             from current point, a segment point set
	*/
	void DFS(Mat& depth, Mat& visit, Point cur, short& threshold, vector<Point>& v);
	/**
	* @brief NonRecursive: prevent stack overflow
	* @param depth     depth data
	* @param visit     marked visited point
	* @param cur       current point position
	* @param pSet      from current point, a segment point set
	*/
	void NonRecursive(Mat& depth, Mat& visit, Point& cur, PointSet& pSet);
	/** 
	* @brief completeDepth complete black points with right/down/right-down direction which has depth value 
	*/
	void completeDepth(Mat& depth);
	/**
	* @brief clear mainRegions_, blackRegions_ and distance_
	*/
	void clear();
private:
	/**
	* @brief regionMerge: merge small region and black points to Main Regions
	* @param depth     depth data
	* @param minSim    0.0 - 1.0
	*/
	void regionMerge(Mat& depth, SegmentSet& segment, SegmentSet& blackRegions, unsigned topk, double minSim);
	/**
	* @brief hullBoundBox: get the each region convexHull's bounding box
	* @param hull  the vertices of Convex hull
	* @return bounding box
	*/
	inline Rect hullBoundBox(PointSet& hull);
	/**
	* @brief isRegionInsideHull: determine if small or black regions belongs to one Main Region
	* @param pSet      small region / black region
	* @param hull      Convex hull
	* @param minSim    0.0 - 1.0
	* @return  in / not in
	*/
	inline bool isRegionInsideHull(PointSet& pSet, PointSet& hull, double minSim);
	inline bool isRegionInsideHull(PointSet& pSet, PointSet& hull, PointSet& seg, double minSim);
	/**
	* @brief calculateConvexHulls: calculate convex hulls and store in this->convexHulls_
	*/
	inline void calculateConvexHulls();
	/**
	* @brief calculateBoundBoxex: calculate boumd boxes and store in this->boundBoxes_
	*/
	inline void calculateBoundBoxex();
	/** 
	* @brief randColor: generate random colors to show 
	*/
	void randColor();

	/** 
	* @brief drawXXX: extend from Draw 
	*/
	virtual void drawRotateRect(Mat& src, RotatedRect& rr);
	virtual void drawConvexHull(Mat& src, PointSet& hull, Scalar color);

	virtual void drawBlack(SegmentSet& blackRegions, Mat& disp, Vec3b& color);
	virtual void draw(SegmentSet& segment, Mat& disp, vector<Vec3b>& colors);
	virtual void drawBoundBox(SegmentSet& segment, vector<double>& distance, Mat& color, Mat& depth, string categoryName = "bottle");
	virtual void drawRegions(SegmentSet& segment, Mat& color, Mat& depth, Mat& disp);

	virtual void drawSobel(Mat& depth);

public:
	// segmentation info 
	SegmentSet mainRegions_; // topk merged main regions
	SegmentSet blackRegions_; // unreliable point region
	vector<PointSet> convexHulls_; // convex hulls_ of mainRegions_
	vector<Rect> boundBoxes_; // bound boxes of mainRegions_
	vector<double> distance_; // the average distance of mainRegions_
	// parameters
	static const vector<Point> _DIRECTIONS_;  // eight directions to extend point
	static const vector<Point> _DIR_; // three directions (right, down, right-down) to complete point
	vector<Vec3b> colors_; // random color to mark segmentation
	Rect RANGE_; // image size to determine which region is out-of-range
	short threshold_; // the max distance from extended point
	unsigned topk_; // initial main regions number
};
