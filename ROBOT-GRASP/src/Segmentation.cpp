#include "Segmentation.hpp"
#include <stack>
using std::stack;

const vector<Point> Segmentation::_DIRECTIONS_ = { Point(1, 0), Point(0, -1),
Point(-1, 0), Point(0, 1),
Point(1, 1), Point(-1, 1),
Point(1, -1), Point(-1, -1) };
const vector<Point> Segmentation::_DIR_ = { Point(1, 0), Point(0, 1), Point(1, 1) };


Segmentation::Segmentation(int width, int height, unsigned k, short t)
{
	RANGE_.width = width;
	RANGE_.height = height;
	threshold_ = t;
	topk_ = k;
	//Generate random colors
	this->randColor();
}

Segmentation::Segmentation(Size sz, unsigned k, short t)
{
	RANGE_.width = sz.width;
	RANGE_.height = sz.height;
	threshold_ = t;
	topk_ = k;
	//Generate random colors
	this->randColor();
}

/*
void Segmentation::Segment(Mat& depth, Mat& color)
{
Mat disp = Mat::zeros(depth.size(), CV_8UC3);
Mat visit = Mat::zeros(depth.size(), CV_8U);
//Region Growing
for (int i = 0; i < RANGE_.width; i++)
for (int j = 0; j < RANGE_.height; j++) {
Point current(i, j);
short value = depth.at<short>(current);
if (!visit.at<char>(current)) {
PointSet v;
v.push_back(current);
visit.at<char>(current) = 255;
DFS(depth, visit, current, threshold_, v);
//insert segment
if (!value)
blackRegions_.push_back(v);
else
mainRegions_.push_back(v);
}
}
//Vec3b xxx(255, 255, 255);
//drawBlack(blackRegions, disp, xxx);
//drawSobel(depth);
sort( mainRegions_.begin(), mainRegions_.end(),
[](const vector<Point>& v1, const vector<Point>& v2){return v1.size() > v2.size();} );
Mat pre = Mat::zeros(depth.size(), CV_8UC3);
draw(mainRegions_, pre, colors_);
imshow("before merging", pre);
regionMerge(depth, mainRegions_, blackRegions_, topk_, 1);

draw(mainRegions_, disp, colors_);
//drawRegions(mainRegions_, color, depth, disp);
drawBoundBox(mainRegions_, distance_, color, depth);

imshow("segmentation", disp);
}
*/

void Segmentation::Segment(Mat& depth, Mat& color)
{
	Mat visit = Mat::zeros(depth.size(), CV_8U);
	//Region Growing
	for (int i = 0; i < RANGE_.width; i++)
		for (int j = 0; j < RANGE_.height; j++) {
			Point current(i, j);
			if (!visit.at<char>(current)) {
				short value = depth.at<short>(current);
				PointSet pSet;
				NonRecursive(depth, visit, current, pSet);
				//insert segment
				if (!value)
					blackRegions_.push_back(pSet);
				else
					mainRegions_.push_back(pSet);
			}
		}
	sort(mainRegions_.begin(), mainRegions_.end(),
		[](const vector<Point>& v1, const vector<Point>& v2){return v1.size() > v2.size(); });
	//show segmentation before and after
	Mat pre = Mat::zeros(depth.size(), CV_8UC3);
	draw(mainRegions_, pre, colors_);
	imshow("before merging", pre);

	regionMerge(depth, mainRegions_, blackRegions_, topk_, 1);
	
	Mat disp = Mat::zeros(depth.size(), CV_8UC3);
	draw(mainRegions_, disp, colors_);
	imshow("segmentation", disp);
	
	//show boundbox
	Mat regions = color.clone();
	calculateConvexHulls();
	calculateBoundBoxex();
	for (auto rect : boundBoxes_) 
		rectangle(regions, rect, Scalar(255, 255, 255), 2);
	imshow("regions", regions);
}

void Segmentation::DFS(Mat &depth, Mat &visit, Point cur, short &threshold, vector<Point> &v)
{
	for (auto d : _DIRECTIONS_){
		Point next = cur + d;
		if (next.inside(RANGE_))
			if (!visit.at<char>(next))
				if (abs(depth.at<short>(next) -depth.at<short>(cur)) < threshold) {
					v.push_back(next);
					visit.at<char>(next) = 255;
					DFS(depth, visit, next, threshold, v);
				}
	}
}

void Segmentation::NonRecursive(Mat &depth, Mat& visit, Point& current, PointSet& pSet)
{
	stack<Point> pstack;
	Point p, next;

	pstack.push(current);
	pSet.push_back(current);
	visit.at<char>(current) = 255;

	while (!pstack.empty()) {
		p = pstack.top();
		pstack.pop();

		for (auto d : _DIRECTIONS_) {
			next = p + d;
			if (!next.inside(RANGE_))
				continue;
			if (visit.at<char>(next))
				continue;
			if (abs(depth.at<short>(next)-depth.at<short>(p)) >= threshold_)
				continue;
			visit.at<char>(next) = 255;
			pstack.push(next);
			pSet.push_back(next);
		}
	}
}

void Segmentation::completeDepth(Mat &depth)
{
	static int WIDTH = RANGE_.width * 2 - 1, HEIGHT = RANGE_.height * 2 - 1;
	short v;
	for (int i = 1; i < WIDTH; i++)
		for (int j = 1; j < HEIGHT; j++) {
			Point current(i, j);
			short& value = depth.at<short>(current);
			if (value)	continue;
			for (auto d : _DIR_) {
				v = depth.at<short>(d + current);
				if (v) {
					//depth.at<short>(current) = v;
					value = v;
					break;
				}
			}
		}
}


void Segmentation::regionMerge(Mat& depth, SegmentSet& segment, SegmentSet& blackRegions, unsigned topk, double minSim)
{
	vector<vector<Point>> hullSet;
	unsigned i, j;

	topk = topk > segment.size() ? segment.size() : topk;
	//find convexHull of each region
	for (auto seg : segment) {
		vector<Point> hull;
		convexHull(seg, hull, false);
		hullSet.push_back(hull);
		double dis = 0;
		for (auto p : seg)
			dis += depth.at<short>(p);
		distance_.push_back(dis / seg.size());
	}
	//fill black regions
	for (auto br : blackRegions)
		for (i = 0; i < topk; i++)
			isRegionInsideHull(br, hullSet[i], segment[i], minSim);
	//isBlackRegionInsideHull(br, hullSet[i], segment[i], minSim, depth, distance[i]);
	//merge
	for (j = topk; j < segment.size(); j++)
		for (i = 0; i < topk; i++)
			isRegionInsideHull(segment[j], hullSet[i], segment[i], minSim);
	//remain topk segment
	segment.resize(topk);
}


inline Rect Segmentation::hullBoundBox(PointSet& hull)
{
	static Point extend(5, 5);
	Point pmax(0, 0), pmin(0x7fffffff, 0x7fffffff);
	for (auto p : hull) {
		if (p.x > pmax.x) pmax.x = p.x;
		if (p.x < pmin.x) pmin.x = p.x;
		if (p.y > pmax.y) pmax.y = p.y;
		if (p.y < pmin.y) pmin.y = p.y;
	}
	return Rect(pmin - extend, pmax + extend) & RANGE_;
}

inline void Segmentation::calculateConvexHulls()
{
	for (auto &seg : mainRegions_) {
		convexHulls_.push_back(PointSet());
		convexHull(seg, convexHulls_.back(), false);
	}
}

inline void Segmentation::calculateBoundBoxex()
{
	for (auto &hull : convexHulls_) 
		boundBoxes_.push_back(hullBoundBox(hull));
}

inline bool Segmentation::isRegionInsideHull(PointSet& pSet, PointSet& hull, double minSim)
{
	unsigned sum = 0;
	for (auto p : pSet)
		if (pointPolygonTest(hull, p, false) >= 0)
			sum++;
	return sum >= minSim * pSet.size();
}

inline bool Segmentation::isRegionInsideHull(PointSet& pSet, PointSet& hull, PointSet& seg, double minSim)
{
	unsigned sum = 0;
	for (auto p : pSet)
		if (pointPolygonTest(hull, p, false) >= 0) {
			seg.push_back(p);
			sum++;
		}
	return sum >= minSim * pSet.size();
}

void Segmentation::randColor()
{
	colors_.push_back(Vec3b(255, 0, 0));
	colors_.push_back(Vec3b(0, 255, 0));
	colors_.push_back(Vec3b(0, 0, 255));
	colors_.push_back(Vec3b(0, 255, 255));
	colors_.push_back(Vec3b(255, 0, 255));
	colors_.push_back(Vec3b(255, 255, 0));
	for (int i = 0; i < 10000; i++)
		colors_.push_back(Vec3b(rand() % 255, rand() % 255, rand() % 255));
}

void Segmentation::clear()
{
	this->mainRegions_.clear();
	this->blackRegions_.clear();
	this->distance_.clear();
	this->convexHulls_.clear();
	this->boundBoxes_.clear();
}

void Segmentation::draw(SegmentSet &segment, Mat &disp, vector<Vec3b> &colors)
{
	int count = 0;
	for (auto seg : segment) {
		//imshow("disp", disp);
		for (auto p : seg)
			disp.at<Vec3b>(p) = colors[count];
		count++;
		//waitKey(-1);
	}

}

void Segmentation::drawBlack(SegmentSet &blackRegions, Mat &disp, Vec3b &color)
{
	for (auto black : blackRegions)
		for (auto p : black)
			disp.at<Vec3b>(p) = color;
}

void Segmentation::drawConvexHull(Mat &src, PointSet &hull, Scalar color)
{
	std::vector<cv::Point>::const_iterator it;
	for (it = hull.begin() + 1; it != hull.end(); it++)
		line(src, *(it - 1), *it, color, 2);
	line(src, hull[0], hull.back(), color, 2);
}

void Segmentation::drawRotateRect(Mat &src, RotatedRect &rr)
{
	Point2f vertices[4];
	rr.points(vertices);
	for (int i = 0; i < 4; i++)
		line(src, vertices[i], vertices[(i + 1) % 4], Scalar(0, 0, 255), 3);
}

void Segmentation::drawSobel(Mat &depth)
{
	Mat sobelx, sobely, sobel, disp;
	cv::Sobel(depth, sobelx, 2, 1, 0);
	cv::Sobel(depth, sobely, 2, 0, 1);
	sobel = abs(sobelx) + abs(sobely);
	//cout << "asd" << endl;
	double min, max;
	cv::minMaxLoc(sobel, &min, &max);
	sobel.convertTo(disp, CV_8U, 255. / max);
	Canny(disp, disp, 0, 20);
	cout << max << endl;
	imshow("sobel", disp);
}

void Segmentation::drawRegions(SegmentSet &segment, Mat &color, Mat &depth, Mat &disp)
{
	vector<Mat> rMat;//(segment.size(), Mat::zeros(color.size(), CV_8UC3));
	for (unsigned i = 0; i < segment.size(); i++) {
		rMat.push_back(Mat::zeros(color.size(), CV_8UC3));
		for (auto p : segment[i]) {
			rMat.back().at<Vec3b>(p) = color.at<Vec3b>(p);
		}
		//draw pointset

		//for (auto p: segment[i])
		//    rMat.back().at<Vec3b>(p) = color.at<Vec3b>(p);

		Mat canny, gray, dgray, poly;
		cv::cvtColor(rMat[i], gray, cv::COLOR_RGB2GRAY);
		cv::medianBlur(gray, gray, 5);
		//imshow("wqewq", gray);
		cv::Canny(gray, canny, 100, 300);
		//cv::dilate(canny, canny, Mat());
		//cv::erode(canny, canny, Mat());

		//imwrite("canny.png", canny);
		//waitKey( - 1);

		vector<Point> vex;
		vector<Point> whitePoint;
		for (int i = 0; i < RANGE_.width; i++) {
			for (int j = 0; j < RANGE_.height; j++) {
				if (canny.at<char>(Point(i, j)))
					whitePoint.push_back(Point(i, j));
			}
		}
		cv::approxPolyDP(whitePoint, vex, 50, true);
		for (auto p : vex) {
			cv::circle(color, p, 3, Scalar(255, 0, 0), 3);
		}

		imshow("xxx", color);
		//waitKey( - 1);


		if (i)  continue;


		/*
		vector<Point> corners;
		cv::goodFeaturesToTrack(canny, corners, 10, 0.01, 60, Mat(), 3, false, 4);
		for (auto p: corners)
		cv::circle(color, p, 2, Scalar(255, 0, 0), 2);
		*/




		//vector<vector<Point>> contours;
		//cv::findContours(canny, contours, RETR_EXTERNAL, 1);
		//cv::drawContours(color, contours,  -1, Scalar(0, 0, 0), 2);


		//lineDection(canny, i);

		imshow(to_string(i), canny);
		//imshow(to_string(i), rMat.back());
	}

}

void Segmentation::drawBoundBox(SegmentSet &segment, vector<double> &distance, Mat &color, Mat &depth, string categoryName)
{
	int count = 0;
	Mat classification = color.clone();

	for (auto seg : segment) {
		vector<Point> hull;
		convexHull(seg, hull, false);
		//RotatedRect rr = cv::minAreaRect(hull);
		Rect boundbox = Segmentation::hullBoundBox(hull);
		rectangle(color, boundbox, Scalar(255, 255, 255), 2);
		count++;
	}
	imshow("regions", color);
	imshow("classification", classification);
	imshow("depth", 65535 / 1200 * depth);
}
