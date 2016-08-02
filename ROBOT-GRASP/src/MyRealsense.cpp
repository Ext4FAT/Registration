#include "PCL.hpp"
#include "Macro.h"
#include "DrawWorld.hpp"
#include "Segmentation.hpp"
#include "HOG-SVM.hpp"
#include "MyRealsense.hpp"

#include "Validation.hpp"

typedef vector<PXCPointF32> PXC2DPointSet;
class Reflect_Result {
public:
	bool isEmpty(){
		return model.size() == 0 && grasp.size() == 0;
	}
public:
	PXC2DPointSet model;
	PXC2DPointSet grasp;
};

atomic_bool wait(false);

ofstream result_out;
vector<string> filenames;
static Rect range = { 0, 0, 640, 480 };

Rect myBoundBox(vector<PXCPointF32> &pointset)
{
	//static Point extend(5, 5);
	Point pmax(0, 0), pmin(0x7fffffff, 0x7fffffff);
	for (auto p : pointset) {
		if (p.x > pmax.x) pmax.x = p.x;
		if (p.x < pmin.x) pmin.x = p.x;
		if (p.y > pmax.y) pmax.y = p.y;
		if (p.y < pmin.y) pmin.y = p.y;
	}
	return Rect(pmin, pmax) & range;
}


// Thread
void myimshow(const string winname, Mat &img)
{
	imshow(winname, img);
}

void threadShow(const string winname, Mat &img)
{
	thread t(myimshow, winname, img);
	t.detach();
}

// Convert Realsense's PXC to PCL's PointCloud
size_t PXC2PCL(PointSet &pSet, vector<PXCPoint3DF32> &vertices, PointCloudNT::Ptr &scene, float scale = 1.f / 300.f)
{
	//vector<PXCPoint3DF32> obj_cloud;
	for (auto& p : pSet) {
		p += p;
		PXCPoint3DF32 ppp = vertices[p.y * 640 + p.x];
		scene->push_back(PointNT());
		PointNT& ps = scene->back();
		ps.x = ppp.x*scale;
		ps.y = ppp.y*scale;
		ps.z = ppp.z*scale;
	}
	return scene->size();
}

// Show boundbox and word
inline void drawText(Mat &img, Rect &boundbox, const string content)
{
	putText(img, content, (boundbox.tl() + boundbox.br()) / 2, 3, 0.6, Scalar(0, 0, 255), 2);
}

// Locate windows position
void placeWindows(int topk)
{
	cv::namedWindow("depth");
	cv::namedWindow("color");
	cv::namedWindow("before merging");
	cv::namedWindow("segmentation");
	cv::namedWindow("classification");
	cv::namedWindow("regions");
	cv::namedWindow("reflect");
	cv::namedWindow("point cloud");
	cv::moveWindow("depth", 0, 0);
	cv::moveWindow("color", 350, 0);
	cv::moveWindow("segmentation", 1050, 0);
	cv::moveWindow("before-merging", 700, 0);
	cv::moveWindow("classification", 350, 300);
	cv::moveWindow("regions", 0, 300);
	cv::moveWindow("reflect", 700, 300);
	cv::moveWindow("point cloud", 0, 600);
	//for (int k = 0; k < topk; k++) {
	//	cv::namedWindow(to_string(k));
	//	cv::moveWindow(to_string(k), (k) * 350, 600);
	//}
}

vector<PXCPointF32> genRegistrationResult(	PXCProjection *projection, 
											PointCloudNT::Ptr &model, 
											Segmentation &myseg,
											vector<PXCPoint3DF32> &vertices, 
											double scale, 
											RegisterParameter &para)
{
	//generate Point Cloud
	PointCloudNT::Ptr mesh(new PointCloudNT);
	PointCloudNT::Ptr model_align(new PointCloudNT);
	PointCloudNT::Ptr grasp_align(new PointCloudNT);
	size_t sz = PXC2PCL(myseg.mainRegions_[0], vertices, mesh, 1.0 / scale);
	cout << "Generate Point Cloud: " << sz << endl;
	//Alignment
	Matrix4f transformation = Registration(model, mesh, model_align, para, true);
	if (transformation == Matrix4f::Identity()) //Alignment failed 
		return{};
	vector<PXCPoint3DF32> result;
	for (auto &pc : *model_align) {
		result.push_back({ scale * pc.x, scale * pc.y, scale * pc.z });
	}
	//reflect
	vector<PXCPointF32> show2d(result.size());
	projection->ProjectCameraToDepth(result.size(), &result[0], &show2d[0]);
	return show2d;
}

Reflect_Result genRegistrationResult(	PXCProjection *projection,
										PointCloudNT::Ptr &model,
										PointCloudT::Ptr &grasp,
										PointSet &segment,
										vector<PXCPoint3DF32> &vertices,
										double scale,
										RegisterParameter &para)
{
	//generate Point Cloud
	PointCloudNT::Ptr mesh(new PointCloudNT);
	PointCloudNT::Ptr model_align(new PointCloudNT);
	PointCloudT::Ptr grasp_align(new PointCloudT);
	size_t sz = PXC2PCL(segment, vertices, mesh, 1.0 / scale);
	cout << "Generate Point Cloud: " << sz << endl;
	//Alignment
	
	//Matrix4f transformation = Registration(model, mesh, model_align, leaf, true);
	

	/** TODO
	*/
	
	Matrix4f transformation;
	if (para.Method == RANSACPLUSICP)
		transformation = RegistrationNoShow(model, mesh, model_align, para);
	else {
		transformation = RegistrationNoShow_ICP(model, mesh, model_align, para);
	}
	//Matrix4f transformation = RegistrationNoShow(model, mesh, model_align, para);
	//Matrix4f transformation = RegistrationNoShow(model, mesh, model_align, leaf);
	
	if (transformation == Matrix4f::Identity()) //Alignment failed 
		return{};

	pcl::transformPointCloud(*grasp, *grasp_align, transformation);
	//Reflect
	pcl::ScopeTime t("[Reflect]");
	Reflect_Result show2d;
	show2d.model.resize(model_align->size());
	show2d.grasp.resize(grasp_align->size());
	vector<PXCPoint3DF32> result;
	for (auto &pc : *model_align) {
		result.push_back({ scale * pc.x, scale * pc.y, scale * pc.z });
	}
	projection->ProjectCameraToDepth(result.size(), &result[0], &show2d.model[0]);
	result.clear();
	for (auto &pc : *grasp_align) {
		result.push_back({ scale * pc.x, scale * pc.y, scale * pc.z });
	}
	projection->ProjectCameraToDepth(result.size(), &result[0], &show2d.grasp[0]);
	return show2d;
}

void showRegistrationResult(vector<PXCPointF32> &show2d, Mat &img, Vec3b color)
{
	for (auto p : show2d) {
		Point pp(p.x, p.y);
		if (pp.inside(Rect(0, 0, 640, 480))) {
			img.at<Vec3b>(pp) = color;
		}
	}
	imshow("reflect", img);
}

bool Reflect(	long framecnt,
				string name,
				Mat& img,
				PXCProjection *projection,
				PointCloudNT::Ptr &model,
				PointCloudT::Ptr &grasp,
				PointSet &segment,
				vector<PXCPoint3DF32> &vertices,
				double scale,
				RegisterParameter &para)
{
	MESSAGE_COUT("[" << framecnt << "]", name);
	Mat color = img.clone();
	//vector<PXCPointF32> show2d = genRegistrationResult(projection_, model, myseg, vertices, PointCloudScale, leaf);
	
	Reflect_Result show2d = genRegistrationResult(projection, model, grasp, segment, vertices, scale, para);

	if (!show2d.isEmpty()) {
		Rect boundbox = myBoundBox(show2d.grasp);
		rectangle(color, boundbox, Scalar(255, 0, 0), 2);
		result_out <<framecnt << "\t" << filenames[framecnt] << "\t" << boundbox << endl;

		//rectangle(color, boundbox, Scalar(0, 0, 255), 2);
		showRegistrationResult(show2d.model, color, Vec3b(255, 0, 255));
		showRegistrationResult(show2d.grasp, color, Vec3b(0, 255, 255));
		return true;
		//if (waitKey(-1) == 27) {
		//	wait = false;
		//	return;
		//}
	}
	wait = false;
	return false;
}


//Dir example : ".\\xxx\\"
MyRealsense::MyRealsense(string& Dir, int width, int height, float fps) 
{
	dir_ = Dir;
	depthDir_ = Dir + "\\depth\\";
	colorDir_ = Dir + "\\rgb\\";
	camera_.width = width;
	camera_.height = height;
	fps_ = fps;
	wait_ = false;
	//Configure RealSense
	configRealsense();
}

//Convert RealSense's PXCImage to Opencv's Mat
Mat MyRealsense::PXCImage2Mat(PXCImage* pxc)
{
	if (!pxc)	return Mat(0, 0, 0);
	PXCImage::ImageInfo info = pxc->QueryInfo();
	PXCImage::ImageData data;
	Mat cvt;
	if (info.format & PXCImage::PIXEL_FORMAT_YUY2) {	//颜色数据
		if (pxc->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_RGB24, &data) < PXC_STATUS_NO_ERROR)
			return  Mat(0, 0, 0);
		cvt = Mat(info.height, info.width, CV_8UC3, (void*)data.planes[0],data.pitches[0]/sizeof(uchar));
	}
	else if (info.format & PXCImage::PIXEL_FORMAT_DEPTH) {//深度数据
		if (pxc->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_DEPTH, &data) < PXC_STATUS_NO_ERROR)
			return  Mat(0, 0, 0);
		cvt = Mat(info.height, info.width, CV_16U, (void*)data.planes[0], data.pitches[0] / sizeof(uchar));	//Mat初始化是按照长宽来定的
	}
	pxc->ReleaseAccess(&data);
	return cvt;
}

//Save PXC Point Cloud to PCD file
int MyRealsense::savePCD(const string filename, PointSet &pSet, vector<PXCPoint3DF32> &vertices)
{
	ofstream ofs(filename);
	ofs << "# .PCD v0.7 - Point Cloud Data file format" << endl;
	ofs << "VERSION 0.7" << endl;
	ofs << "FIELDS x y z" << endl;
	ofs << "SIZE 4 4 4" << endl;
	ofs << "TYPE F F F" << endl;
	ofs << "COUNT 1 1 1" << endl;
	ofs << "WIDTH " << pSet.size() << endl;
	ofs << "HEIGHT 1" << endl;
	ofs << "VIEWPOINT 0 0 0 1 0 0 0" << endl;
	ofs << "POINTS " << pSet.size() << endl;
	ofs << "DATA ascii" << endl;
	double scale = 1. / 300;
	//vector<PXCPoint3DF32> obj_cloud;
	for (auto& p : pSet) {
		p += p;
		PXCPoint3DF32 ppp = vertices[p.y * 640 + p.x];
		ofs << ppp.x*scale << " " << ppp.y*scale << " " << ppp.z*scale << endl;
		//obj_cloud.push_back(vertices[p.y * 640 + p.x]);
	}
	ofs.close();
	return 1;
}


inline string MyRealsense::getSavePath(std::string dir,time_t slot, long framecnt)
{
	std::stringstream ss;
	std::string filename;
	ss << dir << slot << "-" << framecnt << ".png";
	ss >> filename;
	return filename;
}

//Configure Realsense parameters
int MyRealsense::configRealsense()
{
	//Configure RealSense
	pxcsession_ = PXCSession::CreateInstance();
	pxcsm_ = pxcsession_->CreateSenseManager();
	pxcsm_->EnableStream(PXCCapture::STREAM_TYPE_COLOR, camera_.width, camera_.height, fps_);
	pxcsm_->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, camera_.width, camera_.height, fps_);
	//Query Information
	pxcsm_->Init();
	pxcdev_ = pxcsm_->QueryCaptureManager()->QueryDevice();
	if (!pxcdev_) {
		MESSAGE_COUT("ERROR", "Failed to create an SDK SenseManager");
		return -1;
	}
	projection_ = pxcdev_->CreateProjection();
	if (!projection_) {
		MESSAGE_COUT("ERROR", "Failed to create an SDK Projection");
		return -2;
	}
	return 0;
}

int MyRealsense::dataAcquire()
{
	////Detect dir exist, if not, create
	//DIR_NOEXIST_AND_CREATE(dir_);
	//DIR_NOEXIST_AND_CREATE(depthDir_);
	//DIR_NOEXIST_AND_CREATE(colorDir_);

	//Define variable
	Mat color, depth, pointcloud;
	vector<PXCPoint3DF32> vertices(camera_.height*camera_.width);
	PXCSession *pxcsession;
	PXCSenseManager *pxcsm;
	PXCCapture::Device *pxcdev;
	PXCProjection *projection;
	PXCCapture::Sample *sample;
	PXCImage *pxcdepth,*pxccolor;
	


	long framecnt;
	//Configure RealSense

	pxcsession = PXCSession::CreateInstance();
	pxcsm = pxcsession->CreateSenseManager();

	//pxcsm->EnableFace();

	pxcsm->EnableStream(PXCCapture::STREAM_TYPE_COLOR, camera_.width, camera_.height, fps_);
	pxcsm->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, camera_.width, camera_.height, fps_);
	
	//pxcsm->EnableStream(PXCCapture::STREAM_TYPE_COLOR, 0, 0, 0, PXCCapture::Device::STREAM_OPTION_STRONG_STREAM_SYNC);

	//pxcsm->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, 0, 0, 0, PXCCapture::Device::STREAM_OPTION_STRONG_STREAM_SYNC);


	DrawWorld dw(pxcsession, camera_);
	PXCPoint3DF32 light = { .5, .5, 1.0 };



	////开启物体识别
	//pxcsm->EnableObjectRecognition();
	//PXCObjectRecognitionModule *reco = pxcsm->QueryObjectRecognition();
	
	pxcsm->Init();
	pxcdev = pxcsm->QueryCaptureManager()->QueryDevice();

	PXCPointF32 cf = pxcdev->QueryColorFocalLength();
	PXCPointF32 df = pxcdev->QueryDepthFocalLength();
	pxcF32 cfmm = pxcdev->QueryColorFocalLengthMM();
	pxcF32 dfmm = pxcdev->QueryDepthFocalLengthMM();
	PXCPointF32 dpp = pxcdev->QueryDepthPrincipalPoint();
	PXCPointF32 cpp = pxcdev->QueryColorPrincipalPoint();
	PXCPointF32 dview = pxcdev->QueryDepthFieldOfView();
	PXCRangeF32 drange = pxcdev->QueryDepthSensorRange();
	PXCCapture::DeviceInfo ppp;
	pxcdev->QueryDeviceInfo(&ppp);
	cout << cfmm << endl;
	cout << dfmm << endl;
	cout << endl;
	//PXCImage::Rotation rt = ppp.rotation;


	pxcdev->SetDepthConfidenceThreshold(4);
	if (!pxcdev) {
		MESSAGE_COUT("ERROR", "Failed to create an SDK SenseManager");
		return -1;
	}
	projection = pxcdev->CreateProjection();
	if (!projection) {
		MESSAGE_COUT("ERROR", "Failed to create an SDK Projection");
		return -1;
	}

	//calibration
	PXCCalibration *calib = projection->QueryInstance<PXCCalibration>();
	PXCCalibration::StreamCalibration sc;
	PXCCalibration::StreamTransform st;
	calib->QueryStreamProjectionParametersEx(PXCCapture::StreamType::STREAM_TYPE_DEPTH,
		PXCCapture::Device::StreamOption::STREAM_OPTION_DEPTH_PRECALCULATE_UVMAP,
		&sc, &st);
	//calib->QueryStreamProjectionParameters(PXCCapture::StreamType::STREAM_TYPE_DEPTH, &sc, &st);

	cout << endl;
	unsigned topk = 5;
	short threshold = 3;
	Size sz(320, 240);
	Segmentation myseg(sz, topk, threshold);

	//Detect each video frame
	for (framecnt = 1; -1 == waitKey(1); framecnt++) {
		if (pxcsm->AcquireFrame(true) < PXC_STATUS_NO_ERROR)	break;
		//Query the realsense color and depth, and project depth to color
		try{
			sample = pxcsm->QuerySample();
			pxcdepth = sample->depth;
			pxccolor = sample->color;
			pxcdepth = projection->CreateDepthImageMappedToColor(pxcdepth, pxccolor);
			
			//vector<PXCPoint3DF32> vex(camera_.width*camera_.height);
			//projection->QueryVertices(pxcdepth, &vex[0]);
			
			//project to world 

			//pxcStatus sts = projection->QueryVertices(pxcdepth, &vertices[0]);
			//if (sts >= PXC_STATUS_NO_ERROR) {
			//	PXCImage* drawVertices = dw.DepthToWorldByQueryVertices(vertices, pxcdepth, light);
			//	if (drawVertices){
			//		Mat pointcloud = PXCImage2Mat(drawVertices);
			//		imshow("pointcloud", pointcloud);
			//	}
			//}
			

			//waitKey(-1);
			

			////project to world
			//projection->QueryVertices(pxcdepth, &vertices[0]);
			//
			//ofstream ofs("123.txt");
			//for (auto v : vertices) {
			//	if (v.x || v.y || v.z)
			//		ofs << "v " << v.x << " " << v.y << " " << v.z << endl;
			//}
			//cout << endl;



			//pxcdepth = projection->CreateColorImageMappedToDepth(pxcdepth,pxccolor);
			depth = PXCImage2Mat(pxcdepth);
			color = PXCImage2Mat(pxccolor);
			if (!depth.cols || !color.cols)	continue;

			Mat depth2, color2;
			resize(depth, depth2, Size(320, 240));
			resize(color, color2, Size(320, 240));

			imshow("depth", 65535 / 1200 * depth2);
			imshow("color", color2);

			myseg.completeDepth(depth);

			myseg.Segment(depth2, color2);
			
			
			myseg.clear();



			//Segment by Depth
			//bfs(depth,vector<vector<Point>>)

			////Construct the filename to save
			//time_t slot = time(0);
			//std::string cpath = getSavePath(colorDir_, slot, framecnt);
			//std::string dpath = getSavePath(depthDir_, slot, framecnt);

			////Save image
			//imwrite(cpath, color);
			//imwrite(dpath, depth);

			//double dmax, dmin;
			//minMaxLoc(depth, &dmin, &dmax);
			//MESSAGE_COUT("MIN-MAX", dmin << " , " << dmax);

			//Show image

			//resize(color, color, cv::Size(640, 480));
			//resize(depth, depth, cv::Size(640, 480));

			
			//imshow("Depth", 256 * 255 / 1200 * depth);
			//imshow("COLOR", color);
			
			
			//Release Realsense SDK memory and read next frame 
			
			//pxccolor->Release();
			pxcdepth->Release();
			pxcsm->ReleaseFrame();
		}
		catch (Exception e){
			MESSAGE_COUT("ERROR", e.what());
		}
	}
	return 1;

}

int MyRealsense::captureDepthandSave()
{
	//Define variable
	Mat color, depth, pointcloud;
	vector<PXCPoint3DF32> vertices(camera_.height*camera_.width);
	PXCSession *pxcsession;
	PXCSenseManager *pxcsm;
	PXCCapture::Device *pxcdev;
	PXCProjection *projection;
	PXCCapture::Sample *sample;
	PXCImage *pxcdepth, *pxccolor;
	long framecnt;
	//Configure RealSense
	pxcsession = PXCSession::CreateInstance();
	pxcsm = pxcsession->CreateSenseManager();
	pxcsm->EnableStream(PXCCapture::STREAM_TYPE_COLOR, camera_.width, camera_.height, fps_);
	pxcsm->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, camera_.width, camera_.height, fps_);
	//Configure Draw
	PXCPoint3DF32 light = { .5, .5, 1.0 };
	DrawWorld dw(pxcsession, camera_);
	//Query Information
	pxcsm->Init();
	pxcdev = pxcsm->QueryCaptureManager()->QueryDevice();
	if (!pxcdev) {
		MESSAGE_COUT("ERROR", "Failed to create an SDK SenseManager");
		return -1;
	}
	projection = pxcdev->CreateProjection();
	if (!projection) {
		MESSAGE_COUT("ERROR", "Failed to create an SDK Projection");
		return -1;
	}
	//Detect each video frame
	for (framecnt = 1; -1 == waitKey(1); framecnt++) {
		if (pxcsm->AcquireFrame(true) < PXC_STATUS_NO_ERROR)	break;
		//Query the realsense color and depth, and project depth to color
		sample = pxcsm->QuerySample();
		pxcdepth = sample->depth;
		pxccolor = sample->color;
		pxcdepth = projection->CreateDepthImageMappedToColor(pxcdepth, pxccolor);
		//Convert PXCImage to Opencv's Mat and show
		depth = PXCImage2Mat(pxcdepth);
		color = PXCImage2Mat(pxccolor);
		if (!depth.cols || !color.cols)	
			continue;
		imshow("depth", 65535 / 1200 * depth);
		imshow("color", color);
		//Generate point cloud and show
		pxcStatus sts = projection->QueryVertices(pxcdepth, &vertices[0]);
		if (sts >= PXC_STATUS_NO_ERROR) {
			PXCImage* drawVertices = dw.DepthToWorldByQueryVertices(pxcdepth, vertices);
			if (drawVertices){
				pointcloud = PXCImage2Mat(drawVertices);
				imshow("point cloud", pointcloud);
			}
		}


		//Mat depth2, color2;
		//resize(depth, depth2, Size(320, 240));
		//resize(color, color2, Size(320, 240));
		

		if (waitKey(33 ) == ' ')
		{
			imwrite(".\\snap\\depth-" + to_string(framecnt) + ".png", depth);
			imwrite(".\\snap\\depth-scale-" + to_string(framecnt) + ".png", 65535 / 1200*depth);
			imwrite(".\\snap\\color-" + to_string(framecnt) + ".png", color);
		}



		//Release
		pxcdepth->Release();
		pxcsm->ReleaseFrame();
	}
	return 1;
}

int MyRealsense::show(){ return -1; }

int MyRealsense::testRegistration(	const string model_path, 
									const string grasp_path, 
									double PointCloudScale,
									RegisterParameter &para)
{
	//Define variable
	Size showSize = { camera_.width / 2, camera_.height / 2 }; //segement and show size is the half of camera
	Mat color, depth, pointcloud;
	vector<PXCPoint3DF32> vertices(camera_.height*camera_.width);
	PXCCapture::Sample *sample;
	PXCImage *pxcdepth, *pxccolor;
	long framecnt;
	int state = 0;
	//Load 3D Model
	const float leaf = para.leaf;
	PointCloudNT::Ptr model(new PointCloudNT);
	LoadModel(model_path, model);
	Downsample(model, leaf);
	//Load grasping point region
	PointCloudT::Ptr grasp(new PointCloudT);
	loadGraspPcd(grasp_path, grasp);
	//Configure Segmentation
	unsigned topk = 5;
	short threshold = 3;
	Segmentation myseg(showSize.width, showSize.height, topk, threshold);
	//Configure HOG-SVM
	HOG_SVM hog_svm(".\\classification\\HOG-SVM-MODEL.xml");
	vector<string> names = hog_svm.getSubdirName(".\\classification");
	hog_svm.getCategory(names);
	//Configure RealSense
	if ((state = configRealsense()) < 0)
		return state;
	//Configure Draw Point Cloud
	DrawWorld dw(pxcsession_, camera_);
	//Detect each video frame
	placeWindows(topk);
	for (framecnt = 1; (1); framecnt++) {
		if (pxcsm_->AcquireFrame(true) < PXC_STATUS_NO_ERROR)	break;
		//Query the realsense color and depth, and project depth to color
		sample = pxcsm_->QuerySample();
		pxcdepth = sample->depth;
		pxccolor = sample->color;
		pxcdepth = projection_->CreateDepthImageMappedToColor(pxcdepth, pxccolor);
		//Convert PXCImage to Opencv's Mat and show
		depth = PXCImage2Mat(pxcdepth);
		color = PXCImage2Mat(pxccolor);
		if (!depth.cols || !color.cols)	
			continue;
		Mat depth2, color2;
		resize(depth, depth2, showSize);
		resize(color, color2, showSize);
		imshow("color", color2);
		imshow("depth", 65536 / 1200 * depth2);
		//Generate Point Cloud and show
		pxcStatus sts = projection_->QueryVertices(pxcdepth, &vertices[0]);
		if (sts >= PXC_STATUS_NO_ERROR) { // Show Point Cloud
			PXCImage* drawVertices = dw.DepthToWorldByQueryVertices(pxcdepth, vertices);
			if (drawVertices)
				pointcloud = PXCImage2Mat(drawVertices);
		}
		resize(pointcloud, pointcloud, showSize);
		imshow("point cloud", pointcloud);
		//Segment by depth data
		myseg.Segment(depth2, color2);
		//Classification
		int k = 0;
		for (auto &boundbox : myseg.boundBoxes_) {
			Mat region = color2(boundbox);
			//int predict = hog_svm.predict(region);
			//if (predict > 0) {
			//	string name = hog_svm.getCategoryName(predict);
				string name = "bottle";
				rectangle(color2, boundbox, Scalar(0, 0, 255), 2);
				drawText(color2, boundbox, name);
				//show point cloud
				imshow("classification", color2);
				imshow("point cloud", pointcloud);
				//Registration
				//if (waitKey(1) == ' ') {
				if (!wait) {
					wait = true;
					thread t(	Reflect,
								framecnt,
								name,
								color,
								projection_,
								model,
								grasp,
								myseg.mainRegions_[k],
								vertices, 
								PointCloudScale, 
								para);
					t.detach();
				}
			//}
			k++;
			if (k > 0)
				break;
		}
		//for (int k = 0; k < topk; k++){
		//	if (0 < hog_svm.predict(color2(myseg.boundBoxes_[k]))) {
		//		//label rectangle
		//		rectangle(color2, myseg.boundBoxes_[k], Scalar(0, 0, 255), 2);
		//		imshow("classification", color2);
		//		////show point cloud
		//		//Mat show = color2.clone();
		//		//for (auto p : myseg.mainRegions_[k])
		//		//	show.at<Vec3b>(p) = pointcloud.at<Vec3b>(p);
		//		//imshow(to_string(k), show);
		//		//show registration
		//		if (waitKey(1) == ' ') {
		//			cout << "[" << framecnt << "]" << endl;
		//			vector<PXCPointF32> show2d = genRegistrationResult(projection_, model, myseg, vertices, PointCloudScale, leaf);
		//			if (show2d.size()) {
		//				showRegistrationResult(show2d, color);
		//				if (waitKey(-1) == 27)
		//					break;
		//			}
		//		}
		//	}
		//}

		//Release
		if (' ' == waitKey(1))
			waitKey(-1);
		myseg.clear();
		pxcdepth->Release();
		pxcsm_->ReleaseFrame();
	}
	projection_->Release();
	pxcdev_->Release();
	pxcsm_->Release();
	pxcsession_->Release();
	return 1;
}

int MyRealsense::testDataSet(	const string model_path,
								const string grasp_path,
								double PointCloudScale,
								RegisterParameter &para,
								string dir,
								string categoryname,
								int from,
								int seg_index)
{
	//Define variable
	Size showSize = { camera_.width / 2, camera_.height / 2 }; //segement and show size is the half of camera
	Mat color, depth, pointcloud;
	vector<PXCPoint3DF32> vertices(camera_.height*camera_.width);
	//PXCCapture::Sample *sample;
	//PXCImage *pxcdepth, *pxccolor;
	long framecnt;
	int state = 0;
	//Load 3D Model
	const float leaf = para.leaf;
	PointCloudNT::Ptr model(new PointCloudNT);
	LoadModel(model_path, model);
	Downsample(model, leaf);
	//Load grasping point region
	PointCloudT::Ptr grasp(new PointCloudT);
	loadGraspPcd(grasp_path, grasp);
	//Configure Segmentation
	unsigned topk = 5;
	short threshold = 3;
	Segmentation myseg(showSize.width, showSize.height, topk, threshold);
	//Configure HOG-SVM
	HOG_SVM hog_svm(".\\classification\\HOG-SVM-MODEL.xml");
	vector<string> names = hog_svm.getSubdirName(".\\classification");
	hog_svm.getCategory(names);
	//Configure Draw Point Cloud
	DrawWorld dw(pxcsession_, camera_);
	//Detect each video frame
	placeWindows(topk);
	//
	readFromCSV(categoryname, ".\\res\\" + categoryname + ".csv", filenames);
	//filenames = getCurdirFileName(dir + "\\" + categoryname);
	framecnt = 0;

	if (para.Method == RANSACPLUSICP)
		result_out.open(categoryname + ".txt");
	else if (para.Method == ICP_CLASSIC)
		result_out.open("ICP-CLASSIC-" + categoryname + ".txt");
	else if (para.Method == ICP_NOLINEAR)
		result_out.open("ICP-NOLINEAR-" + categoryname + ".txt");
	else if (para.Method == ICP_WITHNORMLS)
		result_out.open("ICP-WITHNORMALS-" + categoryname + ".txt");
		
	for (auto filename : filenames) {
		if (framecnt < from) {
			framecnt++;
			continue;
		}
		// Convert PXCImage to Opencv's Mat and show
		string color_path = dir + "\\" + categoryname + "\\" + filename;
		string depth_path = dir + "\\depth\\" + filename;
		color = imread(color_path, CV_LOAD_IMAGE_UNCHANGED);
		depth = imread(depth_path, CV_LOAD_IMAGE_UNCHANGED);
		if (!depth.cols || !color.cols)
			continue;
		// Convert to PXCimage
		PXCImage::ImageInfo iinfo;
		iinfo.height = depth.rows;
		iinfo.width = depth.cols;
		iinfo.format = PXCImage::PIXEL_FORMAT_DEPTH;
		PXCImage::ImageData idata;
		idata.format = PXCImage::PIXEL_FORMAT_DEPTH;
		idata.pitches[0] = depth.step[0] * sizeof(uchar);
		idata.planes[0] = depth.data;
		PXCImage *pxcdepth = pxcsession_->CreateImage(&iinfo, &idata);
		// Resize and show
		Mat depth2, color2;
		resize(depth, depth2, showSize);
		resize(color, color2, showSize);
		imshow("color", color2);
		imshow("depth", 65536 / 1200 * depth2);
		// Generate Point Cloud and show
		pxcStatus sts = projection_->QueryVertices(pxcdepth, &vertices[0]);
		if (sts >= PXC_STATUS_NO_ERROR) { // Show Point Cloud
			PXCImage* drawVertices = dw.DepthToWorldByQueryVertices(pxcdepth, vertices);
			if (drawVertices)
				pointcloud = PXCImage2Mat(drawVertices);
		}
		resize(pointcloud, pointcloud, showSize);
		imshow("point cloud", pointcloud);
		//Segment by depth data
		myseg.Segment(depth2, color2);
		//Classification
		int k = 0;
		for (auto &boundbox : myseg.boundBoxes_) {
			Mat region = color2(boundbox);
			//int predict = hog_svm.predict(region);
			//if (predict > 0) {
			//	string name = hog_svm.getCategoryName(predict);
			string name = categoryname;
			rectangle(color2, boundbox, Scalar(0, 0, 255), 2);
			drawText(color2, boundbox, name);
			//show point cloud
			imshow("classification", color2);
			imshow("point cloud", pointcloud);
			//Registration
			bool flag = Reflect(	framecnt,
						name,
						color,
						projection_,
						model,
						grasp,
						myseg.mainRegions_[k],
						vertices,
						PointCloudScale,
						para);
			//if (waitKey(1) == ' ') {
			//if (!wait) {
			//	wait = true;
			//	thread t(Reflect,
			//		framecnt,
			//		name,
			//		color,
			//		projection_,
			//		model,
			//		grasp,
			//		myseg.mainRegions_[k],
			//		vertices,
			//		PointCloudScale,
			//		para);
			//	t.detach();
			//}
			//}
			k++;
			//if (k > 1)
			if (k > seg_index)
				break;
		}
		framecnt++;

		MYCUSTOM::mynumber++;

		//for (int k = 0; k < topk; k++){
		//	if (0 < hog_svm.predict(color2(myseg.boundBoxes_[k]))) {
		//		//label rectangle
		//		rectangle(color2, myseg.boundBoxes_[k], Scalar(0, 0, 255), 2);
		//		imshow("classification", color2);
		//		////show point cloud
		//		//Mat show = color2.clone();
		//		//for (auto p : myseg.mainRegions_[k])
		//		//	show.at<Vec3b>(p) = pointcloud.at<Vec3b>(p);
		//		//imshow(to_string(k), show);

		//		//show registration
		//		if (waitKey(1) == ' ') {
		//			cout << "[" << framecnt << "]" << endl;
		//			vector<PXCPointF32> show2d = genRegistrationResult(projection_, model, myseg, vertices, PointCloudScale, leaf);
		//			if (show2d.size()) {
		//				showRegistrationResult(show2d, color);
		//				if (waitKey(-1) == 27)
		//					break;
		//			}
		//		}
		//	}
		//}

		//Release
		if (' ' == waitKey(1))
			waitKey(-1);
		myseg.clear();
		pxcdepth->Release();
		//pxcsm_->ReleaseFrame();
	}
	//projection_->Release();
	//pxcdev_->Release();
	//pxcsm_->Release();
	//pxcsession_->Release();
	return 1;

}



int MyRealsense::testFromValidDataSet(	const string model_path,
										const string grasp_path,
										double PointCloudScale,
										RegisterParameter &para,
										string dir,
										string categoryname,
										string csvname) 
{
	//Define variable
	Size showSize = { camera_.width / 2, camera_.height / 2 }; //segement and show size is the half of camera
	Mat color, depth, pointcloud;
	vector<PXCPoint3DF32> vertices(camera_.height*camera_.width);
	//PXCCapture::Sample *sample;
	//PXCImage *pxcdepth, *pxccolor;
	long framecnt;
	int state = 0;
	//Load 3D Model
	const float leaf = para.leaf;
	PointCloudNT::Ptr model(new PointCloudNT);
	LoadModel(model_path, model);
	Downsample(model, leaf);
	//Load grasping point region
	PointCloudT::Ptr grasp(new PointCloudT);
	loadGraspPcd(grasp_path, grasp);
	//Configure Segmentation
	unsigned topk = 5;
	short threshold = 3;
	Segmentation myseg(showSize.width, showSize.height, topk, threshold);
	//Configure HOG-SVM
	HOG_SVM hog_svm(".\\classification\\HOG-SVM-MODEL.xml");
	vector<string> names = hog_svm.getSubdirName(".\\classification");
	hog_svm.getCategory(names);
	//Configure Draw Point Cloud
	DrawWorld dw(pxcsession_, camera_);
	//Detect each video frame
	placeWindows(topk);
	return 1;
}





/*
int Dataset::show()
{
	//Define variable
	Mat color, depth, pointcloud;
	vector<PXCPoint3DF32> vertices(camera_.height*camera_.width);
	PXCSession *pxcsession;
	PXCSenseManager *pxcsm;
	PXCCapture::Device *pxcdev;
	PXCProjection *projection;
	PXCCapture::Sample *sample;
	PXCImage *pxcdepth, *pxccolor;
	long framecnt;
	//Configure RealSense
	pxcsession = PXCSession::CreateInstance();
	pxcsm = pxcsession->CreateSenseManager();
	pxcsm->EnableStream(PXCCapture::STREAM_TYPE_COLOR, camera_.width, camera_.height, fps_);
	pxcsm->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, camera_.width, camera_.height, fps_);
	//Configure Draw
	PXCPoint3DF32 light = { .5, .5, 1.0 };
	DrawWorld dw(pxcsession, camera_);
	//Configure Segmentation
	unsigned topk = 5;
	short threshold = 3;
	Segmentation myseg(320, 240, topk, threshold);
	placeWindows(topk);
	//Query Information
	pxcsm->Init();
	pxcdev = pxcsm->QueryCaptureManager()->QueryDevice();
	//PXCPointF32 cf = pxcdev->QueryColorFocalLength();
	//PXCPointF32 df = pxcdev->QueryDepthFocalLength();
	//pxcF32 cfmm = pxcdev->QueryColorFocalLengthMM();
	//pxcF32 dfmm = pxcdev->QueryDepthFocalLengthMM();
	//PXCPointF32 dpp = pxcdev->QueryDepthPrincipalPoint();
	//PXCPointF32 cpp = pxcdev->QueryColorPrincipalPoint();
	//PXCPointF32 dview = pxcdev->QueryDepthFieldOfView();
	//PXCRangeF32 drange = pxcdev->QueryDepthSensorRange();
	//PXCCapture::DeviceInfo ppp;
	//pxcdev->QueryDeviceInfo(&ppp);
	//cout << cfmm << endl;
	//cout << dfmm << endl;
	//cout << endl;

	//setthreshold
	//pxcdev->SetDepthConfidenceThreshold(4);

	if (!pxcdev) {
		MESSAGE_COUT("ERROR", "Failed to create an SDK SenseManager");
		return -1;
	}
	projection = pxcdev->CreateProjection();
	if (!projection) {
		MESSAGE_COUT("ERROR", "Failed to create an SDK Projection");
		return -1;
	}
	//calibration
	//PXCCalibration *calib = projection->QueryInstance<PXCCalibration>();
	//PXCCalibration::StreamCalibration sc;
	//PXCCalibration::StreamTransform st;
	//calib->QueryStreamProjectionParametersEx(PXCCapture::StreamType::STREAM_TYPE_DEPTH,
	//	PXCCapture::Device::StreamOption::STREAM_OPTION_DEPTH_PRECALCULATE_UVMAP,
	//	&sc, &st);
	//cout << endl;

	//Load3dModel
	std::string model_path = "C:\\Users\\win10\\Desktop\\Realsense-PCL\\ROBOT\\model\\teacup.pcd";
	std::string grasp_path = "C:\\Users\\win10\\Desktop\\Realsense-PCL\\ROBOT\\model\\grasp.obj";
	PointCloudNT::Ptr object(new PointCloudNT);
	PointCloudNT::Ptr grasp(new PointCloudNT);
	FeatureCloudT::Ptr object_features(new FeatureCloudT);
	loadModel(model_path, object, object_features, 0.01f);
	loadGrasp(grasp_path, grasp);




	//Detect each video frame
	for (framecnt = 1; -1 == waitKey(1); framecnt++) {
		if (pxcsm->AcquireFrame(true) < PXC_STATUS_NO_ERROR)	break;
		//Query the realsense color and depth, and project depth to color
		sample = pxcsm->QuerySample();
		pxcdepth = sample->depth;
		pxccolor = sample->color;
		pxcdepth = projection->CreateDepthImageMappedToColor(pxcdepth, pxccolor);

		pxcStatus sts = projection->QueryVertices(pxcdepth, &vertices[0]);
		if (sts >= PXC_STATUS_NO_ERROR) {
			PXCImage* drawVertices = dw.DepthToWorldByQueryVertices(vertices, pxcdepth, light);
			if (drawVertices){
				pointcloud = PXCImage2Mat(drawVertices);
				//imshow("pointcloud", pointcloud);
			}
		}

		depth = PXCImage2Mat(pxcdepth);
		color = PXCImage2Mat(pxccolor);
		if (!depth.cols || !color.cols)	continue;

		Mat depth2, color2;
		resize(depth, depth2, Size(320, 240));
		resize(color, color2, Size(320, 240));

		Mat show = color2.clone();

		myseg.completeDepth(depth);

		imshow("depth", 65535 / 1200 * depth2);
		imshow("color", color2);

		myseg.Segment(depth2, color2);
		resize(pointcloud, pointcloud, Size(320, 240));


		int S = 1;
		//myseg.mainRegions_.size();
		for (int k = 0; k < S; k++){
			for (auto p : myseg.mainRegions_[k]) {
				show.at<Vec3b>(p) = pointcloud.at<Vec3b>(p);
			}
			imshow(to_string(k), show);
		}


		//vector<PXCPoint3DF32> obj_cloud;
		//static Point dir[5] = { { 0, 0 }, { 0, 1 }, { 0, -1 }, { 1, 0 }, { -1, 0 } };
		//static Rect range(0, 0, 640, 480);
		//for (int k = 0; k < S; k++){
		//	Mat show = Mat::zeros(Size(640, 480), CV_8U);

		//	for (auto p : myseg.mainRegions_[k]) {
		//		cout << p.x <<" "<< p.y << endl;
		//		for (auto d : dir) {
		//			Point pp = p * 2 + d;
		//			if (!pp.inside(range)) continue;
		//			cout << p.x << " " << p.y << endl;
		//			obj_cloud.push_back(vertices[pp.y * 640 + pp.x]);
		//			show.at<Vec3b>(pp) = pointcloud.at<Vec3b>(pp);
		//		}
		//	}
		//	imshow(to_string(k), show);
		//}


		//myseg.Segment(depth, color);

		if (waitKey(1) == ' ') {

			//savePCD(to_string(framecnt) + ".pcd", myseg.mainRegions_[0], vertices);
			//cout << "Output completed" << endl;


			cout << "[" << framecnt << "]" << endl;
			std::string outputf = "real.pcd";
			//generate Point Cloud
			PointCloudNT::Ptr scene(new PointCloudNT);
			size_t sz = PXC2PCL(myseg.mainRegions_[0], vertices, scene);
			std::cout << "Generate Point Cloud: " << sz << std::endl;	
			//savePCD(outputf, myseg.mainRegions_[0], vertices);
			
			//Alignment
			float leaf = 0.01f;
			float maxcd = 4.f * leaf;
			float sim = 0.9f;
			FeatureCloudT::Ptr scene_features(new FeatureCloudT);
			preprocessScene(scene, scene_features, leaf);

			for (int i = 0; i < 20; i++, maxcd += leaf){
				vector<PXCPoint3DF32> result;
				pointCloudAlignment(object, object_features, scene, scene_features, grasp, result, leaf, maxcd, sim);
				//world2color
				if (result.size()) {
					///
					///
					///
					vector<PXCPointF32> show2d(grasp->size());
					projection->ProjectCameraToDepth(result.size(), &result[0], &show2d[0]);
					//projection->ProjectCameraToColor(result.size(), &result[0], &show2d[0]);
					cout << result.size() << endl;
					cout << show2d.size() << endl;
					//Mat mydisp= cv::Mat::zeros(color.size(), CV_8U);
					Mat mydisp = color.clone();
					for (auto p : show2d) {
						Point pp(p.x, p.y);
						//cout << pp.x <<" "<< pp.y << endl;

						if (pp.inside(Rect(0, 0, 640, 480))) {
							//if (pp.inside(Rect(320,240,200,100))){
							mydisp.at<Vec3b>(pp) = Vec3b(255, 0, 255);
							//cv::circle(color, pp, 10, Scalar(255, 0, 255), 10);
						}
					}
					imshow("res", mydisp);
					imwrite(".\\res\\mydisp" + to_string(framecnt) +"-"+to_string(i) + ".png", mydisp);
					imwrite(".\\res\\mycolor" + to_string(framecnt) +"-"+to_string(i)+ ".png", color);
					result.clear();
					show2d.clear();

				}
			}

			//if (waitKey(-1) == 27)
			//	break;
		}
		//Release
		myseg.clear();
		pxcdepth->Release();
		pxcsm->ReleaseFrame();
	}
	return 1;
}
*/





//int Dataset::show()
//{
//	//Define variable
//	Mat color, depth, pointcloud;
//	vector<PXCPoint3DF32> vertices(camera_.height*camera_.width);
//	PXCSession *pxcsession;
//	PXCSenseManager *pxcsm;
//	PXCCapture::Device *pxcdev;
//	PXCProjection *projection;
//	PXCCapture::Sample *sample;
//	PXCImage *pxcdepth, *pxccolor;
//	long framecnt;
//	//Configure RealSense
//	pxcsession = PXCSession::CreateInstance();
//	pxcsm = pxcsession->CreateSenseManager();
//	pxcsm->EnableStream(PXCCapture::STREAM_TYPE_COLOR, camera_.width, camera_.height, fps_);
//	pxcsm->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, camera_.width, camera_.height, fps_);
//	//Configure Draw
//	PXCPoint3DF32 light = { .5, .5, 1.0 };
//	DrawWorld dw(pxcsession, camera_);
//	//Configure Segmentation
//	unsigned topk = 5;
//	short threshold = 3;
//	Segmentation myseg(320, 240, topk, threshold);
//	placeWindows(topk);
//	//Query Information
//	pxcsm->Init();
//	pxcdev = pxcsm->QueryCaptureManager()->QueryDevice();
//
//	PXCPointF32 cf = pxcdev->QueryColorFocalLength();
//	PXCPointF32 df = pxcdev->QueryDepthFocalLength();
//	pxcF32 cfmm = pxcdev->QueryColorFocalLengthMM();
//	pxcF32 dfmm = pxcdev->QueryDepthFocalLengthMM();
//	PXCPointF32 dpp = pxcdev->QueryDepthPrincipalPoint();
//	PXCPointF32 cpp = pxcdev->QueryColorPrincipalPoint();
//	PXCPointF32 dview = pxcdev->QueryDepthFieldOfView();
//	PXCRangeF32 drange = pxcdev->QueryDepthSensorRange();
//	PXCCapture::DeviceInfo ppp;
//	pxcdev->QueryDeviceInfo(&ppp);
//	cout << cfmm << endl;
//	cout << dfmm << endl;
//	cout << endl;
//
//	
//	//setthreshold
//	//pxcdev->SetDepthConfidenceThreshold(4);
//
//
//	if (!pxcdev) {
//		MESSAGE_COUT("ERROR", "Failed to create an SDK SenseManager");
//		return -1;
//	}
//	projection = pxcdev->CreateProjection();
//	if (!projection) {
//		MESSAGE_COUT("ERROR", "Failed to create an SDK Projection");
//		return -1;
//	}
//
//	//calibration
//	//PXCCalibration *calib = projection->QueryInstance<PXCCalibration>();
//	//PXCCalibration::StreamCalibration sc;
//	//PXCCalibration::StreamTransform st;
//	//calib->QueryStreamProjectionParametersEx(PXCCapture::StreamType::STREAM_TYPE_DEPTH,
//	//	PXCCapture::Device::StreamOption::STREAM_OPTION_DEPTH_PRECALCULATE_UVMAP,
//	//	&sc, &st);
//	//cout << endl;
//
//	//Detect each video frame
//	for (framecnt = 1; -1 == waitKey(1); framecnt++) {
//		if (pxcsm->AcquireFrame(true) < PXC_STATUS_NO_ERROR)	break;
//		//Query the realsense color and depth, and project depth to color
//		sample = pxcsm->QuerySample();
//		pxcdepth = sample->depth;
//		pxccolor = sample->color;
//		pxcdepth = projection->CreateDepthImageMappedToColor(pxcdepth, pxccolor);
//
//		pxcStatus sts = projection->QueryVertices(pxcdepth, &vertices[0]);
//		if (sts >= PXC_STATUS_NO_ERROR) {
//			PXCImage* drawVertices = dw.DepthToWorldByQueryVertices(vertices, pxcdepth, light);
//			if (drawVertices){
//				pointcloud = PXCImage2Mat(drawVertices);
//				//imshow("pointcloud", pointcloud);
//			}
//		}
//
//		depth = PXCImage2Mat(pxcdepth);
//		color = PXCImage2Mat(pxccolor);
//		if (!depth.cols || !color.cols)	continue;
//
//		Mat depth2, color2;
//		resize(depth, depth2, Size(320, 240));
//		resize(color, color2, Size(320, 240));
//
//		myseg.completeDepth(depth);
//
//		imshow("depth", 65535 / 1200 * depth2);
//		imshow("color", color2);
//		Mat show = color2.clone();
//			
//		myseg.Segment(depth2, color2);
//		resize(pointcloud, pointcloud, Size(320, 240));
//			
//		
//		int S = 1;
//		myseg.mainRegions_.size();
//
//
//		//set<Point> obj_point ;
//		vector<PXCPoint3DF32> obj_cloud;
//		static Point dir[5] = { { 0, 0 }, { 0, 1 }, { 0, -1 }, { 1, 0 }, { -1, 0 } };
//		static Rect range(0, 0, 640, 480);
//		for (int k = 0; k < S; k++){
//			for (auto p : myseg.mainRegions_[k]) {
//				show.at<Vec3b>(p) = pointcloud.at<Vec3b>(p);
//				//for (auto d : dir) {
//				//	Point pp = p * 2 + d;
//				//	if (!pp.inside(range)) continue;
//				//	obj_point.insert(pp);
//				//}
//			}
//			imshow(to_string(k), show);
//		}
//
//
//		//vector<PXCPoint3DF32> obj_cloud;
//		//static Point dir[5] = { { 0, 0 }, { 0, 1 }, { 0, -1 }, { 1, 0 }, { -1, 0 } };
//		//static Rect range(0, 0, 640, 480);
//		
//		//for (int k = 0; k < S; k++){
//		//	Mat show = Mat::zeros(Size(640, 480), CV_8U);
//		//	
//		//	for (auto p : myseg.mainRegions_[k]) {
//		//		//cout << p.x <<" "<< p.y << endl;
//		//		for (auto d : dir) {
//		//			Point pp = p*2 + d;
//		//			if (!pp.inside(range)) continue;
//		//			//cout << p.x << " " << p.y << endl;
//		//			obj_cloud.push_back(vertices[pp.y * 640 + pp.x]);
//		//			//show.at<Vec3b>(pp) = pointcloud.at<Vec3b>(pp);
//		//		}
//		//	}			
//		//	imshow(to_string(k), show);
//		//}
//
//
//		myseg.Segment(depth, color);
//
//		myseg.clear();
//			
//		if (waitKey(1) == ' ') {
//			ofstream ofs("real-cloud.obj");
//			for (auto o : obj_cloud) {
//				ofs << "v " << o.x << " " << o.y << " " << o.x << endl;
//			}
//			waitKey(-1);
//		}
//
//		//Release
//		pxcdepth->Release();
//		pxcsm->ReleaseFrame();
//	}
//	return 1;
//}

