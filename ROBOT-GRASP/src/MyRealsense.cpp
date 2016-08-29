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


string Method2String(METHOD m){
	static vector<string> methods = {	"ICP_CLASSIC",
										"ICP_NOLINEAR",
										"ICP_WITHNORMLS",
										"RANSACPLUSICP",
										"SAC_IA"	};
	if (m > methods.size() || m < 0)
		return "";
	return methods.at(m);
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
	for (int k = 0; k < topk; k++) {
		cv::namedWindow(to_string(k));
		cv::moveWindow(to_string(k), (k) * 350, 600);
	}
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
	/** TODO
	*/
	Matrix4f transformation;
	if (para.Method == RANSACPLUSICP)
		transformation = RegistrationNoShow(model, mesh, model_align, para);
	else {
		transformation = RegistrationNoShow_ICP(model, mesh, model_align, para);
	}
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

void showRegistrationResult(	vector<PXCPointF32> &show2d, 
								Mat &img, 
								Vec3b color)
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
		showRegistrationResult(show2d.model, color, Vec3b(255, 0, 255));
		showRegistrationResult(show2d.grasp, color, Vec3b(0, 255, 255));
		return true;
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
	configRealsense();
}

//Find bounding box
Rect MyRealsense::myBoundBox(vector<PXCPointF32> &pointset)
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
	for (auto& p : pSet) {
		p += p;
		PXCPoint3DF32 ppp = vertices[p.y * 640 + p.x];
		ofs << ppp.x*scale << " " << ppp.y*scale << " " << ppp.z*scale << endl;
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

int MyRealsense::captureColorandDepth()
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
		//Show image
		Mat depth2, color2;
		resize(depth, depth2, Size(320, 240));
		resize(color, color2, Size(320, 240));
		imshow("color", color2);
		imshow("depth", 65535 / 1200 * depth2);
		//Press `space` to capture data
		if (waitKey(33 ) == ' ') {
			imwrite(".\\snap\\depth-" + to_string(framecnt) + ".png", depth);
			imwrite(".\\snap\\color-" + to_string(framecnt) + ".png", color);
		}
		//Release
		pxcdepth->Release();
		pxcsm->ReleaseFrame();
	}
	return 1;
}

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
	LoadGraspPcd(grasp_path, grasp);
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
		for (int k = 0; k < topk; k++){
			Rect roi = myseg.boundBoxes_[k];
			Mat region = color2(roi);
			int predict = hog_svm.predict(region);
			if (0 < predict) {
				//label rectangle
				string name = hog_svm.getCategoryName(predict);
				rectangle(color2, roi, Scalar(0, 0, 255), 2);
				imshow("classification", color2);
				//show registration
				if (waitKey(1) == ' ') {
					cout << "[" << framecnt << "]" << endl;
					if (!wait) {
						wait = true;
						thread t(Reflect,
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
	LoadGraspPcd(grasp_path, grasp);
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
	//Read test data filename 
	readFromCSV(categoryname, ".\\res\\" + categoryname + ".csv", filenames);
	//Open outfile.txt
	result_out.open(Method2String(para.Method) + "-" + categoryname + ".txt");
	//Read source data
	framecnt = 0;
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
		clock_t start = clock();
		myseg.Segment(depth2, color2);
		MESSAGE_COUT("[Segmentation]\t", 1.0*(clock() - start) / CLOCKS_PER_SEC);
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
			k++;
			if (k > seg_index)
				break;
		}
		framecnt++;
		MYCUSTOM::mynumber++;
		//Release
		if (' ' == waitKey(1))
			waitKey(-1);
		myseg.clear();
		pxcdepth->Release();
		//pxcsm_->ReleaseFrame();
	}
	result_out.close();
	//projection_->Release();
	//pxcdev_->Release();
	//pxcsm_->Release();
	//pxcsession_->Release();
	return 1;

}


