#include "MyRealsense.hpp"

/******************************main**********************************/
int main(int argc, char** argv)
{
	string save_dir_path = ".\\";
	string model_path = "C:\\Users\\IDLER\\Documents\\Visual Studio 2013\\Projects\\PCL\\ROBOT-NEW\\ROBOT-GRASP\\ROBOT-GRASP\\model\\bottle\\bottle.pcd";
	double scale = argc > 1 ? atof(argv[1]) : 400.0;
	//F200
	MyRealsense robot(save_dir_path, 640, 480, 30);
	robot.testRegistration(model_path, scale);
	
	
	
	return 0;
}
