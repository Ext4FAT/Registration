#include "DrawWorld.hpp"

DrawWorld::DrawWorld(PXCSession* s, PXCSizeI32 user_size)
{
	session_ = s;
	depthSize_ = user_size;
	light_ = { .5, .5, 1.0 };
	init();
}

DrawWorld::~DrawWorld()
{
	session_->Release();
	drawVertices_->Release();
	vertices_.clear();
}

void DrawWorld::init()
{
	PXCImage::ImageInfo drawVerticesInfo;
	memset(&drawVerticesInfo, 0, sizeof(drawVerticesInfo));
	drawVerticesInfo.width = depthSize_.width;
	drawVerticesInfo.height = depthSize_.height;
	drawVerticesInfo.format = PXCImage::PIXEL_FORMAT_RGB32;
	drawVertices_ = 0;
	drawVertices_ = session_->CreateImage(&drawVerticesInfo);
}


void DrawWorld::norm(PXCPoint3DF32 &v)
{
	float len = sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
	if (len != 0) 
		v.x /= len; v.y /= len; v.z /= len;
}

PXCPoint3DF32 DrawWorld::cross(PXCPoint3DF32 &v0, PXCPoint3DF32 &v1)
{
	PXCPoint3DF32 vec = { v0.y*v1.z - v0.z*v1.y, v0.z*v1.x - v0.x*v1.z, v0.x*v1.y - v0.y*v1.x };
	return vec;
}

float DrawWorld::dot(PXCPoint3DF32 &v0, PXCPoint3DF32 &v1)
{
	return v0.x*v1.x + v0.y*v1.y + v0.z*v1.z;
}



PXCImage* DrawWorld::DepthToWorldByQueryVertices(PXCImage *depth, vector<PXCPoint3DF32>& vertices)
{
	if (!drawVertices_)	return 0;
	PXCImage::ImageInfo drawVerticesInfo = drawVertices_->QueryInfo();
	PXCImage::ImageData drawVerticesDat;
	if (PXC_STATUS_NO_ERROR > drawVertices_->AcquireAccess(PXCImage::ACCESS_WRITE, drawVerticesInfo.format, &drawVerticesDat))
		return 0;

	/* Retrieve vertices */
	float brightness = 200.f;

	PXCImage::ImageInfo dinfo = depth->QueryInfo();
	pxcBYTE* pdrawVerticesDat = drawVerticesDat.planes[0];



	for (pxcI32 y = 1; y < dinfo.height - 1; y++) {
		for (pxcI32 x = 1; x < dinfo.width - 1; x++) {
			pdrawVerticesDat[4 * x] = pdrawVerticesDat[4 * x + 1] = pdrawVerticesDat[4 * x + 2] = 0;
			float fLight = 0;
			PXCPoint3DF32 v0 = vertices[y * dinfo.width + x];
			if (v0.z != 0) {
				//取v0四角点
				PXCPoint3DF32 v1 = vertices[(y - 1) * dinfo.width + (x - 1)];
				PXCPoint3DF32 v2 = vertices[(y - 1) * dinfo.width + (x + 1)];
				PXCPoint3DF32 v3 = vertices[(y + 1) * dinfo.width + (x + 1)];
				PXCPoint3DF32 v4 = vertices[(y + 1) * dinfo.width + (x - 1)];
				//转成向量
				v1.x = v1.x - v0.x; v1.y = v1.y - v0.y; v1.z = v1.z - v0.z;
				v2.x = v2.x - v0.x; v2.y = v2.y - v0.y; v2.z = v2.z - v0.z;
				v3.x = v3.x - v0.x; v3.y = v3.y - v0.y; v3.z = v3.z - v0.z;
				v4.x = v4.x - v0.x; v4.y = v4.y - v0.y; v4.z = v4.z - v0.z;
				//求出四个法向量
				PXCPoint3DF32 vn1 = cross(v1, v2); norm(vn1);
				PXCPoint3DF32 vn2 = cross(v2, v3); norm(vn2);
				PXCPoint3DF32 vn3 = cross(v3, v4); norm(vn3);
				PXCPoint3DF32 vn4 = cross(v4, v1); norm(vn4);
				//
				vn1.x += vn2.x + vn3.x + vn4.x;
				vn1.y += vn2.y + vn3.y + vn4.y;
				vn1.z += vn2.z + vn3.z + vn4.z;
				norm(vn1);
				//
				norm(light_);
				fLight = dot(vn1, light_);
			}

			pdrawVerticesDat[4 * x] = pdrawVerticesDat[4 * x + 1] = pdrawVerticesDat[4 * x + 2] = pxcBYTE(abs(fLight) * 255);
			pdrawVerticesDat[4 * x + 3] = MAXBYTE;
		}
		//换到下一个行
		pdrawVerticesDat += drawVerticesDat.pitches[0];
	}
	drawVertices_->ReleaseAccess(&drawVerticesDat);
	return drawVertices_;
}
PXCImage* DrawWorld::SegmentationWorld(PXCImage *depth, vector<PXCPoint3DF32> &vertices, PointSet &seg)
{
	if (!drawVertices_)	return 0;
	PXCImage::ImageInfo drawVerticesInfo = drawVertices_->QueryInfo();
	PXCImage::ImageData drawVerticesDat;
	if (PXC_STATUS_NO_ERROR > drawVertices_->AcquireAccess(PXCImage::ACCESS_WRITE, drawVerticesInfo.format, &drawVerticesDat))
		return 0;
	/* Retrieve vertices */
	float brightness = 200.f;
	PXCImage::ImageInfo dinfo = depth->QueryInfo();
	pxcBYTE* pdrawVerticesDat = drawVerticesDat.planes[0];
	memset(pdrawVerticesDat, -1, 4 * dinfo.width*dinfo.height);
	for (auto p : seg) {
		float fLight = 0;
		pxcI32 x = p.x * 2, y = p.y * 2;
		PXCPoint3DF32 v0 = vertices[y * dinfo.width + x];
		if (v0.z != 0) {
			//取v0四角点
			PXCPoint3DF32 v1 = vertices[(y - 1) * dinfo.width + (x - 1)];
			PXCPoint3DF32 v2 = vertices[(y - 1) * dinfo.width + (x + 1)];
			PXCPoint3DF32 v3 = vertices[(y + 1) * dinfo.width + (x + 1)];
			PXCPoint3DF32 v4 = vertices[(y + 1) * dinfo.width + (x - 1)];
			//转成向量
			v1.x = v1.x - v0.x; v1.y = v1.y - v0.y; v1.z = v1.z - v0.z;
			v2.x = v2.x - v0.x; v2.y = v2.y - v0.y; v2.z = v2.z - v0.z;
			v3.x = v3.x - v0.x; v3.y = v3.y - v0.y; v3.z = v3.z - v0.z;
			v4.x = v4.x - v0.x; v4.y = v4.y - v0.y; v4.z = v4.z - v0.z;
			//求出四个法向量
			PXCPoint3DF32 vn1 = cross(v1, v2); norm(vn1);
			PXCPoint3DF32 vn2 = cross(v2, v3); norm(vn2);
			PXCPoint3DF32 vn3 = cross(v3, v4); norm(vn3);
			PXCPoint3DF32 vn4 = cross(v4, v1); norm(vn4);
			//
			vn1.x += vn2.x + vn3.x + vn4.x;
			vn1.y += vn2.y + vn3.y + vn4.y;
			vn1.z += vn2.z + vn3.z + vn4.z;
			norm(vn1);
			//
			norm(light_);
			fLight = dot(vn1, light_);
		}
		pdrawVerticesDat[4 * x] = pdrawVerticesDat[4 * x + 1] = pdrawVerticesDat[4 * x + 2] = pxcBYTE(abs(fLight) * 255);
	}
	drawVertices_->ReleaseAccess(&drawVerticesDat);
	return drawVertices_;
}