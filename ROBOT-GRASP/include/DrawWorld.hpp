#include "Opencv.hpp"

#include <pxcimage.h>
#include <pxcprojection.h>
#include <pxcsession.h>


#define MAXBYTE 0xff

class DrawWorld{
public:	
	DrawWorld::DrawWorld(PXCSession* s, PXCSizeI32 user_size);
	DrawWorld::~DrawWorld();

	PXCImage* DepthToWorldByQueryVertices(vector<PXCPoint3DF32>& vertices, PXCImage *depth, PXCPoint3DF32 light);
	PXCImage* SegmentationWorld(std::vector<PXCPoint3DF32>& vertices, PXCImage *depth, PXCPoint3DF32 light, vector<cv::Point> seg);

private:
	inline void init();
	inline void norm(PXCPoint3DF32 &v);
	inline PXCPoint3DF32 cross(PXCPoint3DF32 &v0, PXCPoint3DF32 &v1);
	inline float dot(PXCPoint3DF32 &v0, PXCPoint3DF32 &v1);

private:
		PXCSession* session;
		PXCImage *drawVertices;
		std::vector<PXCPoint3DF32> vertices;
		PXCSizeI32 depthSize;

};