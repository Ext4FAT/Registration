#include "FileOperation.hpp"

#include <windows.h>
#include <vector>
#include <string>

//Get current dir filepath
std::vector<std::string> FileOperation::getCurdirFilePath(std::string dirPath)
{
	WIN32_FIND_DATA ffd;
	HANDLE hFind = INVALID_HANDLE_VALUE;
	std::vector<std::string> files;
	hFind = FindFirstFile((dirPath + "\\*").c_str(), &ffd);
	do
	{
		if (!(ffd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
			files.push_back(dirPath + "\\" + ffd.cFileName);
	} while (FindNextFile(hFind, &ffd) != 0);
	FindClose(hFind);
	return files;
}

//Get current dir filename
std::vector<std::string> FileOperation::getCurdirFileName(std::string dirPath)
{
	WIN32_FIND_DATA ffd;
	HANDLE hFind = INVALID_HANDLE_VALUE;
	std::vector<std::string> files;
	hFind = FindFirstFile((dirPath + "\\*").c_str(), &ffd);
	do
	{
		if (!(ffd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
			files.push_back(ffd.cFileName);
	} while (FindNextFile(hFind, &ffd) != 0);
	FindClose(hFind);
	return files;
}

//Get subdir 
std::vector<std::string> FileOperation::getSubdirName(std::string dirPath)
{
	WIN32_FIND_DATA ffd;
	HANDLE hFind = INVALID_HANDLE_VALUE;
	std::vector<std::string> files;
	hFind = FindFirstFile((dirPath + "\\*").c_str(), &ffd);
	do
	{
		if (ffd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
			files.push_back(ffd.cFileName);
			if (files.back() == "." || files.back() == "..")
				files.pop_back();
		}
	} while (FindNextFile(hFind, &ffd) != 0);
	FindClose(hFind);
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