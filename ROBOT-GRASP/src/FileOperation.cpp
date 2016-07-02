#include <io.h>
#include "FileOperation.hpp"

//Get current dir filepath
std::vector<std::string> FileOperation::getCurdirFilePath(std::string dirPath)
{
	long  hFile = 0;
	struct _finddata_t fileinfo;
	std::string p;
	std::vector<std::string> files;
	if ((hFile = _findfirst(p.assign(dirPath).append("\\*").c_str(), &fileinfo)) != -1) {
		do {
			if (!(fileinfo.attrib & _A_SUBDIR))
				files.push_back(p.assign(dirPath).append("\\").append(fileinfo.name));
				//dirPath + "\\" + fileinfo.name
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
	return files;
}

//Get current dir filename
std::vector<std::string> FileOperation::getCurdirFileName(std::string dirPath)
{
	long  hFile = 0;
	struct _finddata_t fileinfo;
	std::string p;
	std::vector<std::string> files;
	if ((hFile = _findfirst(p.assign(dirPath).append("\\*").c_str(), &fileinfo)) != -1) {
		do {
			if (!(fileinfo.attrib & _A_SUBDIR))
				files.push_back(fileinfo.name);
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
	return files;
}

//Get file name from file path;
std::string FileOperation::findFileName(std::string path)
{
	int i, last;
	for (i = 0; path[i]; i++)
		if (path[i] == '\\')
			last = i;
	return path.substr(last + 1);
}