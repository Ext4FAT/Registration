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

	PXCImage* DepthToWorldByQueryVertices(PXCImage *depth, vector<PXCPoint3DF32> &vertices);
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