#include "FileOperation.hpp"
#include "Opencv.hpp"
/************************************************************************/
/* namespace cv and cv::ml                                                         */
/************************************************************************/
using cv::HOGDescriptor;
using cv::Algorithm;
using cv::TermCriteria;
using cv::Ptr;
using cv::ml::TrainData;
using cv::ml::ROW_SAMPLE;
using cv::ml::SVM;

/*******************************************************************************
*   Label image data with path when testing, output which image error          *
*******************************************************************************/
class GroundTruth {
public:
    int label;
    string imgname;
    GroundTruth(int c, string path){
        label = c;
        imgname = path;
    };
};

/************************************************************************/
/* Mapping category name and index                                      */
/************************************************************************/
class Category {
public:
	Category(){}
	void clear() {
		index2name.clear();
		name2index.clear();
		background = "";
	}
public:
	map<int, string> index2name;
	map<string, int> name2index;
	string background = "Background";
};


/************************************************************************/
/* Utilize HOG as feature, and SVM as machine learning model            */
/************************************************************************/

class HOG_SVM : public FileOperation
{
public:
    /** @brief default constructor */
    HOG_SVM();
    /**
     * @brief HOG_SVM: load xml file as model
     * @param model_path    xml loacation
     */
    HOG_SVM(const string model_path);
    /**
     * @brief loadModel: load xml file as model
     * @param model_path    xml loacation
     * @return success/ fail
     */
    bool loadModel(const string model_path);
    /**
     * @brief extractFeature: extract HOG feature
     * @param Img   input source
     * @param mrs   scaled size
     * @return      feature vector
     */
    Mat extractFeature(Mat Img, Size mrs);
	/**
	* @brief getCategory: get category from sub dir names
	* @param subdirs	categorynames
	* @return			category number
	*/
	int getCategory(vector<string> &subdirs);
    /**
     * @brief getDataSet: get dataset
     * @param data_path     image location
     * @return      feature matrix
     */
    int getDataSet(vector<string> &data_path, double gt);
    Mat getDataSet(vector<string> &data_path, vector<GroundTruth>& gt, int c);
    /**
     * @brief setSvmParameter: set training Parameter
     * @param sv_num    max support vectors number
     * @param c_r_type  classification/ regression
     * @param kernel    linear / gussian / ploy ...
     * @param gamma     if gussian need to set
     * @return          1
     */
    int setSvmParameter(int sv_num, int c_r_type, int kernel, double gamma);
    /**
     * @brief training: training with svm
     * @param trainSet  train data mat
     * @param label     label mat
     * @param save      save model or not
     * @param dir       save path
     * @return          1
     */
    int training(Mat& trainSet, Mat& label, bool save, string dir);
    /**
     * @brief testing: if need to test, you can use this function
     * @param testSet   test matirx
     * @param gt        groundtruth, the othe impletementation is to show which are predicted error
     * @return          1
     */
    int testing(Mat& testSet, float gt);
    int testing(Mat& testSet, vector<GroundTruth> gt);
	/**
	* @brief BinaryClassification: just train two classification.
	* @param pos_path/neg_path	postive_data/negative_data
	* @return  SUCCESS / FAILED
	*/
	int BinaryClassification(string pos_path, string neg_path);
    /**
     * @brief EndToEnd: the whole process, training and testing
     * @param data_path datapath
     * @return  error rate in the test data
     */
    float EndToEnd(string data_path);
    /**
     * @brief predict: predict label in practical application
     * @param image    image / region need to classify
     * @return label
     */
    float predict(Mat& image);
	/**
	* @brief releaseTrainSet: clear trainMat_ and labels_
	*/
	inline void releaseTrainSet();
	/**
	* @brief clearALL: release Pointer svm_, clear catergory_, trainMat_ and labels_
	*/
	inline void clearALL();
	/**
	* @brief get members const
	*/
	const Category& category(){
		return catergory_;
	}
	const Ptr<SVM>& svm() {
		return svm_;
	}
private:
	Ptr<SVM> svm_;
	Category catergory_;
	Mat trainMat_;
	Mat labels_;
};
