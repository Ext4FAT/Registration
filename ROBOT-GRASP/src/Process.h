#include <opencv2/opencv.hpp>
#include <vector>

/*
class Segmentation{
	//const vector<Point> _DIRECTIONS_ = { Point(1, 0), Point(0, -1), Point(-1, 0), Point(0, 1), Point(1, 1), Point(-1, 1), Point(1, -1), Point(-1, -1) };
	//const Rect _RANGE_ = { 0, 0, 320, 240 };

	void DFS(Mat& depth, Mat& visit, Point cur, short& threshold, vector<Point>& v);

	inline Rect hullBoundBox(vector<Point> hull)
	{
		static Point extend(5, 5);
		Point pmax(0, 0), pmin(1000, 1000);
		for (auto p : hull) {
			if (p.x > pmax.x) pmax.x = p.x;
			if (p.x < pmin.x) pmin.x = p.x;
			if (p.y > pmax.y) pmax.y = p.y;
			if (p.y < pmin.y) pmin.y = p.y;
		}
		return Rect(pmin - extend, pmax + extend) & _RANGE_;
	}



};
*/



int bfs(cv::Mat& depth, std::vector<std::vector<cv::Point>>& pointSet, short t);

int testDepth(int fps = 30, int skip = 2);

void showMeRoi(int category);