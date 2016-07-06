#include "MyRealsense.hpp"
#include "Macro.h"

int main(int argc, char** argv)
{
	MESSAGE_COUT("USAGE\n", "\tROBOR-GRASP [Scale] [Model Path]");
	string save_dir_path = ".\\";
	double scale = argc > 1 ? atof(argv[1]) : 400.0;
	string model_path = argc > 2 ? argv[2] : ".\\model\\bottle\\bottle.pcd";
	//F200
	MyRealsense robot(save_dir_path, 640, 480, 30);
	robot.testRegistration(model_path, scale);
	return 0;
}
