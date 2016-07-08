
#include "HOG-SVM.hpp"
#include "Macro.h"

HOG_SVM::HOG_SVM()
{
    svm_ = SVM::create();
}

HOG_SVM::HOG_SVM(const string model_path)
{
    svm_ = Algorithm::load<SVM>(model_path);
}

inline void HOG_SVM::releaseTrainSet()
{
	trainMat_.release();
	labels_.release();
}

inline void HOG_SVM::clearALL()
{
	svm_.release();
	catergory_.clear();
	releaseTrainSet();
}

bool HOG_SVM::loadModel(const string model_path)
{
    bool flag = true;
    try{
        svm_ = Algorithm::load<SVM>(model_path);
    }
    catch (std::exception e){
		MESSAGE_COUT("ERROR", e.what());
        flag = false;
    }
    return flag;
}

Mat HOG_SVM::extractFeature(Mat Img, Size mrs)
{
    /**
     * @brief HOG_SVM::extractFeature
        The story behind 1764
        For example
        window size is 64x64, block size is 16x16 and block setp is 8x8£¬cell size is 8x8,
        the block number window contained is (£¨64-16£©/8+1)*((64-16)/8+1) = 7*7 = 49,
        the cell number each block contained is (16/8)*(16/8) = 4
        every cell can project 9 bin, and each bin related to 9 vector
        so feature_dim  = B x C x N, and caulated result is  1764
        (B is each window's blocks number, C is every block's cell number, n is bin number)
     */
    resize(Img, Img, mrs);
    HOGDescriptor *hog = new HOGDescriptor(cvSize(64, 64), cvSize(16, 16), cvSize(8, 8), cvSize(8, 8), 9);
    std::vector<float> descriptors;
    hog->compute(Img, descriptors, Size(1, 1), Size(0, 0));
    return Mat(descriptors).t();
}

int HOG_SVM::getCategory(vector<string> &subdirs)
{
	int index = 1;
	for (auto &sd: subdirs) {
		catergory_.name2index[sd] = index;
		catergory_.index2name[index] = sd;
		index++;
	}
	return index;
}

int HOG_SVM::getDataSet(vector<string> &data_path, double gt)
{
    int nImgNum = data_path.size();
    int success = 0;
	MESSAGE_COUT("GET DATA ", gt);
    for (auto &path: data_path){
        Mat src = imread(path);
        if (src.cols && src.rows){
            Mat post = extractFeature(src, Size(64, 64));
            trainMat_.push_back(post);
			MESSAGE_COUT("PROCESS " << ++success, findFileName(path));
        }
    }
	Mat tmp = Mat::ones(success, 1, CV_32SC1) * gt;
	labels_.push_back(tmp);
    return success;
}

Mat HOG_SVM::getDataSet(std::vector<std::string> &data_path, std::vector<GroundTruth>& gt, int c)
{
    int nImgNum = data_path.size();
    int success = 0;
    Mat data_mat;	//feature matrix
    Mat src;
    string imgname;
    for (int i = 0; i < nImgNum; i++){
        src = imread(data_path[i]);
        if (src.cols && src.rows){
            imgname = FileOperation::findFileName(data_path[i]);
			MESSAGE_COUT("PROCESS", imgname << "\t" << success++);
            Mat post = extractFeature(src, Size(64, 64));
            data_mat.push_back(post);
            gt.push_back(GroundTruth(c, imgname));
        }
    }
    return data_mat;
}

int HOG_SVM::setSvmParameter(int sv_num, int c_r_type, int kernel, double gamma)
{
    TermCriteria criteria = TermCriteria(CV_TERMCRIT_EPS, sv_num, FLT_EPSILON);	//max support vectocr 200
    svm_->setType(c_r_type);
    svm_->setKernel(kernel);
    if (kernel == SVM::RBF)	svm_->setGamma(gamma);
    svm_->setTermCriteria(criteria);
    return 1;
}

int HOG_SVM::training(Mat& trainSet, Mat& label, bool save,std::string dir)
{
    setSvmParameter(200, SVM::C_SVC, SVM::LINEAR, 0);
    Ptr<TrainData> traindata = cv::ml::TrainData::create(trainSet, ROW_SAMPLE, label);
    svm_->train(traindata);
    if (save){
		svm_->save(dir + "HOG-SVM-MODEL.xml");
    }
    return 1;
}

int HOG_SVM::testing(Mat& testSet, float gt)
{
    int error = 0;
    int postnum = testSet.rows;
    Mat res = Mat::zeros(postnum, 1, CV_32FC1);
    svm_->predict(testSet, res);
    for (int i = 0; i < postnum; i++)
        if (res.at<float>(i, 0) != gt)
            error++;
    std::cout << error << "/" << postnum << std::endl;
    return error;
}

int HOG_SVM::testing(Mat& testSet, std::vector<GroundTruth> gt)
{
    int error = 0;
    int postnum = testSet.rows;
    Mat res = Mat::zeros(postnum, 1, CV_32FC1);
    svm_->predict(testSet, res);
    for (int i = 0; i < postnum; i++)
        if (res.at<float>(i, 0) != gt[i].label){
			MESSAGE_COUT("ERROR", gt[i].imgname << "\t" << gt[i].label);
            error++;
        }
	MESSAGE_COUT("RESULT", error << "/" << postnum);
    return error;
}

float HOG_SVM::predict(Mat& image)
{
    if (!image.rows)	return	-1;
    Mat gray;
    cvtColor(image, gray, CV_BGR2GRAY);
    Mat post = extractFeature(gray, Size(64, 64));
    gray.release();
    return svm_->predict(post);
}

int HOG_SVM::BinaryClassification(string pos_path, string neg_path)
{
	vector<string> pospaths = getCurdirFilePath(pos_path + "\\");
	vector<string> negpaths = getCurdirFilePath(neg_path + "\\");
	getDataSet(pospaths, 1);
	getDataSet(negpaths, -1);
	//training model
	return training(trainMat_, labels_, true, ".\\IDLER-DESKTOP-ITEMS\\");
}


float HOG_SVM::EndToEnd(string data_path)
{
	//Trainset path
	vector<string> subdirs = getSubdirName(data_path);
	getCategory(subdirs);
	//Get trainset
	for (auto &subdir : subdirs) {
		vector<string> imgpaths = getCurdirFilePath(data_path + subdir + "\\");
		getDataSet(imgpaths, catergory_.name2index[subdir]);
	}
	//training model
	training(trainMat_, labels_, true, data_path);

    return 1.0f;
}
