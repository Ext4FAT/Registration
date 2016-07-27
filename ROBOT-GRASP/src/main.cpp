#include "MyRealsense.hpp"
#include "Macro.h"

int main(int argc, char** argv)
{
	MESSAGE_COUT("USAGE", "\tROBOR-GRASP [Scale] [Model Path]");
	string save_dir_path = ".\\";
	double scale = argc > 1 ? atof(argv[1]) : 400.0;
	//string categoryname = "bottle2";
	string categoryname = argc > 2 ? argv[2] : "bottle" ;
	string model_path = ".\\model\\" + categoryname + "\\" + categoryname + "-scaled.pcd";
	string grasp_path = ".\\model\\" + categoryname + "\\" + categoryname + "-grasp-scaled.pcd";
	string dir = argc > 3 ? argv[3] : "C:\\Users\\IDLER\\Desktop\\DATASET\\DataSet";
	//Parameter
	RegisterParameter para;
	if (argc > 4)
		para.leaf = atof(argv[4]);
	if (argc > 5)
		para.NumberOfSamples = atoi(argv[5]);
	if (argc > 6)
		para.CorrespondenceRandomness = atoi(argv[6]);
	if (argc > 7)
		para.SimilarityThreshold = atof(argv[7]);
	if (argc > 8)
		para.MaxCorrespondence = atof(argv[8]);
	if (argc > 9)
		para.InlierFraction = atof(argv[9]);
	//F200
	MyRealsense robot(save_dir_path, 640, 480, 30);

	//robot.testRegistration(model_path, grasp_path, scale, para);

	robot.testDataSet(model_path, grasp_path, scale, para, dir, categoryname);

	return 0;
}
