/************************************************************************/
/* namespace cv                                                         */
/************************************************************************/
#include "Opencv.hpp"

/************************************************************************/
/* namespace pxc                                                        */
/************************************************************************/
#include <pxcimage.h>
#include <pxcprojection.h>
#include <pxcsession.h>

/************************************************************************/
/* User defined                                                         */
/************************************************************************/
#define MAXBYTE 0xff


class DrawWorld{
public:	
	DrawWorld::DrawWorld(PXCSession* s, PXCSizeI32 user_size);
	DrawWorld::~DrawWorld();
	/**
	*  @brief DepthToWorldByQueryVertices: reflect depth image to Point Cloud and show 
	*  @param depth		input depth image
	*  @param vertices	output generated Point Cloud by Realsense
	*  @return			Point Cloud show image
	*/
	PXCImage* DepthToWorldByQueryVertices(PXCImage *depth, vector<PXCPoint3DF32> &vertices);
	/**
	*  @brief SegmentationWorld: reflect Segmentation to Point Cloud and show 
	*  @param depth		input depth image
	*  @param vertices	output generated Point Cloud by Realsense
	*  @param seg		Segmentation Point Set
	*  @return			Point Cloud show image
	*/
	PXCImage* SegmentationWorld(PXCImage *depth, vector<PXCPoint3DF32> &vertices, PointSet &seg);

private:
	inline void init();
	inline void norm(PXCPoint3DF32 &v);
	inline PXCPoint3DF32 cross(PXCPoint3DF32 &v0, PXCPoint3DF32 &v1);
	inline float dot(PXCPoint3DF32 &v0, PXCPoint3DF32 &v1);

private:
		PXCSession* session_;
		PXCImage *drawVertices_;
		vector<PXCPoint3DF32> vertices_;
		PXCSizeI32 depthSize_;
		PXCPoint3DF32 light_;

};