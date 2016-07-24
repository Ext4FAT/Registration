#include "MyRealsense.hpp"
#include "Macro.h"

int main(int argc, char** argv)
{
	MESSAGE_COUT("USAGE", "\tROBOR-GRASP [Scale] [Model Path]");
	string save_dir_path = ".\\";
	double scale = argc > 1 ? atof(argv[1]) : 400.0;
	string model_path = argc > 2 ? argv[2] : ".\\model\\bottle\\bottle.pcd";
	string grasp_path = argc > 3 ? argv[3] : ".\\model\\bottle\\bottle-grasp-scaled.pcd";
	
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

	robot.testRegistration(model_path, grasp_path, scale, para);
	return 0;
}
