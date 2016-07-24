/************************************************************************/
/* namespace cv                                                         */
/************************************************************************/
#include "Opencv.hpp"
#include "FileOperation.hpp"
#include "PCL.hpp"

/************************************************************************/
/* namespace pxc                                                        */
/************************************************************************/
#include <pxcimage.h>
#include <pxcsession.h>
#include <pxcsensemanager.h>
#include <pxcprojection.h>

//thread
#include <thread>
#include <boost\atomic.hpp>
using boost::atomic_bool;
using std::thread;

/*******************************************************************************
*   Realsense Operation.													   *
*******************************************************************************/
class MyRealsense: public FileOperation
{
public:
	/**
	*  @brief testRegistration: End-TO-End Test RANSAC+ICP
	*  @param Dir					save color/depth data, for example: ".\\xxx\\"
	*  @param width, height, fps    camera setting
	*/
	MyRealsense(string& Dir, int width, int height, float fps = 60);
	/**
	*  @brief dataAcquire: Acquire color or depth data
	*  @return	0: success; -1: pxcdevice error; -2: pxcproject cann't create 
	*/
	int dataAcquire();
	/**
	*  @brief configRealsense: configure Realsense and get ready for starting
	*  @return	0: success; -1: pxcdevice error; -2: pxcproject cann't create
	*/
	int configRealsense();
	/**
	*  @brief show: demo with realtime capture video stream
	*/
	int show();
	/**
	*  @brief testRegistration: End-TO-End Test RANSAC+ICP
	*  @param model_path   which model to load (.pcd file format)
	*  @param grasp_path   which model to load (.obj file format)
	*/
	int testRegistration(	const string model_path, 
							const string grasp_path, 
							double PointCloudScale,
							RegisterParameter &para	);
	/**
	*  @brief PXCImage2Mat: Convert RealSense's PXCImage to Opencv's Mat
	*/
	cv::Mat PXCImage2Mat(PXCImage* pxc);
private:
	int captureDepthandSave();
	string getSavePath(const string dir, time_t slot, long framecnt);
	int savePCD(const string filename, PointSet &pSet, vector<PXCPoint3DF32> &vertices);

private:
	// DataAcquire Setting
	string dir_;
	string depthDir_;
	string colorDir_;
	// Realsense
	PXCSession *pxcsession_ = 0;
	PXCSenseManager *pxcsm_ = 0;
	PXCCapture::Device *pxcdev_ = 0;
	PXCProjection *projection_ = 0;
	PXCSizeI32 camera_;
	pxcF32 fps_;
	// Concurrent
	atomic_bool wait_;

};