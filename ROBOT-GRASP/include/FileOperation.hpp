/*******************************************************************************
Some Files and Os Operation, such as open, read, write, etc
*******************************************************************************/
#pragma once

#include <vector>

class FileOperation{
public:
	virtual std::vector<std::string> getCurdirFilePath(std::string dirPath);
	virtual std::vector<std::string> getCurdirFileName(std::string dirPath);
	virtual std::vector<std::string> getSubdirName(std::string dirPath);
	virtual std::string findFileName(std::string path);
};