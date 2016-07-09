#include "PCL.hpp"
#include "Macro.h"
#include "DrawWorld.hpp"
#include "Segmentation.hpp"
#include "HOG-SVM.hpp"
#include "MyRealsense.hpp"

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

void placeWindows(int topk)
{
	cv::namedWindow("depth");
	cv::namedWindow("color");
	cv::namedWindow("before merging");
	cv::namedWindow("segmentation");
	cv::namedWindow("classification");
	cv::namedWindow("regions");
	cv::namedWindow("reflect");
	cv::moveWindow("depth", 0, 0);
	cv::moveWindow("color", 350, 0);
	cv::moveWindow("segmentation", 1050, 0);
	cv::moveWindow("before-merging", 700, 0);
	cv::moveWindow("classification", 350, 300);
	cv::moveWindow("regions", 0, 300);
	cv::moveWindow("reflect", 1600, 0);
	//for (int k = 0; k < topk; k++) {
	//	cv::namedWindow(to_string(k));
	//	cv::moveWindow(to_string(k), (k) * 350, 600);
	//}
}

vector<PXCPointF32> genRegistrationResult(PXCProjection *projection, PointCloudNT::Ptr &model, Segmentation &myseg, vector<PXCPoint3DF32> &vertices, double scale, float leaf)
{
	//generate Point Cloud
	PointCloudNT::Ptr mesh(new PointCloudNT);
	PointCloudNT::Ptr model_align(new PointCloudNT);
	size_t sz = PXC2PCL(myseg.mainRegions_[0], vertices, mesh, 1.0 / scale);
	cout << "Generate Point Cloud: " << sz << endl;
	//Alignment
	Matrix4f transformation = Registration(model, mesh, model_align, leaf, true);
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

void showRegistrationResult(vector<PXCPointF32> &show2d, Mat &color)
{
	for (auto p : show2d) {
		Point pp(p.x, p.y);
		if (pp.inside(Rect(0, 0, 640, 480))) {
			color.at<Vec3b>(pp) = Vec3b(255, 0, 255);
		}
	}
	imshow("reflect", color);
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
}

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

int MyRealsense::outputPCD(const string filename, PointSet &pSet, vector<PXCPoint3DF32> &vertices)
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

inline std::string MyRealsense::getSavePath(std::string dir,time_t slot, long framecnt)
{
	std::stringstream ss;
	std::string filename;
	ss << dir << slot << "-" << framecnt << ".png";
	ss >> filename;
	return filename;
}

int MyRealsense::dataAcquire()
{
	////Detect dir exist, if not, create
	//DIR_NOEXIST_AND_CREATE(dir_);
	//DIR_NOEXIST_AND_CREATE(depthDir_);
	//DIR_NOEXIST_AND_CREATE(colorDir_);

	//Define variable
	Mat color, depth, display;
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
			//		Mat display = PXCImage2Mat(drawVertices);
			//		imshow("display", display);
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
	Mat color, depth, display;
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

		pxcStatus sts = projection->QueryVertices(pxcdepth, &vertices[0]);
		if (sts >= PXC_STATUS_NO_ERROR) {
			PXCImage* drawVertices = dw.DepthToWorldByQueryVertices(pxcdepth, vertices);
			if (drawVertices){
				display = PXCImage2Mat(drawVertices);
				//imshow("display", display);
			}
		}

		depth = PXCImage2Mat(pxcdepth);
		color = PXCImage2Mat(pxccolor);
		if (!depth.cols || !color.cols)	continue;

		//Mat depth2, color2;
		//resize(depth, depth2, Size(320, 240));
		//resize(color, color2, Size(320, 240));
		
		imshow("depth", 65535 / 1200*depth);
		imshow("color", color);
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

int MyRealsense::show(){}

int MyRealsense::testRegistration(const string model_path, double PointCloudScale)
{
	//Define variable
	Size showSize = { camera_.width / 2, camera_.height / 2 }; //segement and show size is the half of camera
	Mat color, depth, display;
	vector<PXCPoint3DF32> vertices(camera_.height*camera_.width);
	PXCCapture::Sample *sample;
	PXCImage *pxcdepth, *pxccolor;
	long framecnt;
	//Load3dModel
	const float leaf = 0.01f;
	PointCloudNT::Ptr model(new PointCloudNT);
	LoadModel(model_path, model);
	Downsample(model, leaf);
	//Configure Segmentation
	unsigned topk = 5;
	short threshold = 3;
	Segmentation myseg(showSize.width / 2, showSize.height / 2, topk, threshold);
	//Configure HOG-SVM
	HOG_SVM hog_svm(".\\classification\\HOG-SVM-MODEL.xml");
	//Configure RealSense
	pxcsession_ = PXCSession::CreateInstance();
	pxcsm_ = pxcsession_->CreateSenseManager();
	pxcsm_->EnableStream(PXCCapture::STREAM_TYPE_COLOR, camera_.width, camera_.height, fps_);
	pxcsm_->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, camera_.width, camera_.height, fps_);
	//Configure Draw Point Cloud
	DrawWorld dw(pxcsession_, camera_);
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
	//Detect each video frame
	placeWindows(topk);
	for (framecnt = 1; (1); framecnt++) {
		if (pxcsm_->AcquireFrame(true) < PXC_STATUS_NO_ERROR)	break;
		//Query the realsense color and depth, and project depth to color
		sample = pxcsm_->QuerySample();
		pxcdepth = sample->depth;
		pxccolor = sample->color;
		pxcdepth = projection_->CreateDepthImageMappedToColor(pxcdepth, pxccolor);
		//Generate Point Cloud
		pxcStatus sts = projection_->QueryVertices(pxcdepth, &vertices[0]);
		if (sts >= PXC_STATUS_NO_ERROR) {
			//show
			PXCImage* drawVertices = dw.DepthToWorldByQueryVertices(pxcdepth, vertices);
			if (drawVertices){
				display = PXCImage2Mat(drawVertices);
				//imshow("display", display);
			}
		}
		//Convert to Opencv Mat
		depth = PXCImage2Mat(pxcdepth);
		color = PXCImage2Mat(pxccolor);
		if (!depth.cols || !color.cols)	continue;
		Mat depth2, color2;
		resize(depth, depth2, showSize);
		resize(color, color2, showSize);
		imshow("color", color2);
		imshow("depth", 65536 / 1200 * depth2);
		resize(display, display, showSize);
		//show segment Point Cloud
		myseg.Segment(depth2, color2);
		for (int k = 0; k < topk; k++){
			if (1 == hog_svm.predict(color2(myseg.boundBoxes_[k]))) {
				//label rectangle
				rectangle(color2, myseg.boundBoxes_[k], Scalar(0, 0, 255), 2);
				imshow("classification", color2);
				////show point cloud
				//Mat show = color2.clone();
				//for (auto p : myseg.mainRegions_[k])
				//	show.at<Vec3b>(p) = display.at<Vec3b>(p);
				//imshow(to_string(k), show);
				//show registration
				if (waitKey(1) == ' ') {
					cout << "[" << framecnt << "]" << endl;
					vector<PXCPointF32> show2d = genRegistrationResult(projection_, model, myseg, vertices, PointCloudScale, leaf);
					if (show2d.size()) {
						showRegistrationResult(show2d, color);
						if (waitKey(-1) == 27)
							break;
					}
				}

			}
		}
		//Release
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





/*
int Dataset::show()
{
	//Define variable
	Mat color, depth, display;
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
				display = PXCImage2Mat(drawVertices);
				//imshow("display", display);
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
		resize(display, display, Size(320, 240));


		int S = 1;
		//myseg.mainRegions_.size();
		for (int k = 0; k < S; k++){
			for (auto p : myseg.mainRegions_[k]) {
				show.at<Vec3b>(p) = display.at<Vec3b>(p);
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
		//			show.at<Vec3b>(pp) = display.at<Vec3b>(pp);
		//		}
		//	}
		//	imshow(to_string(k), show);
		//}


		//myseg.Segment(depth, color);

		if (waitKey(1) == ' ') {

			//outputPCD(to_string(framecnt) + ".pcd", myseg.mainRegions_[0], vertices);
			//cout << "Output completed" << endl;


			cout << "[" << framecnt << "]" << endl;
			std::string outputf = "real.pcd";
			//generate Point Cloud
			PointCloudNT::Ptr scene(new PointCloudNT);
			size_t sz = PXC2PCL(myseg.mainRegions_[0], vertices, scene);
			std::cout << "Generate Point Cloud: " << sz << std::endl;	
			//outputPCD(outputf, myseg.mainRegions_[0], vertices);
			
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
//	Mat color, depth, display;
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
//				display = PXCImage2Mat(drawVertices);
//				//imshow("display", display);
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
//		resize(display, display, Size(320, 240));
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
//				show.at<Vec3b>(p) = display.at<Vec3b>(p);
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
//		//			//show.at<Vec3b>(pp) = display.at<Vec3b>(pp);
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

