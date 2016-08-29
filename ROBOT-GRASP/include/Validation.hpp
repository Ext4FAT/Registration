#include "Common.hpp"
#include "Opencv.hpp"
#include <fstream>
using std::ifstream;

class Validdata{
public:
	Validdata(int num, string filename, double jaccard) {
		this->num = num;
		this->filename = filename;
		this->jaccard = jaccard;
	}
public:
	int num;
	string filename;
	double jaccard;
};

vector<string> splitStr(string str, char c)
{
	vector<string> ret;
	string tmp = "";
	for (int i = 0; i < str.length(); ++i) {
		if (str[i] == c) {
			ret.push_back(tmp);
			tmp = "";
		}
		else {
			tmp += str[i];
		}
	}
	ret.push_back(tmp);
	return ret;
}

int readFromCSV(string category, string csvname)
{
	ifstream readin(csvname);
	string row, filename;
	int num;
	double jaccard;
	readin >> row;
	while (readin >> row) {
		vector<string> v = splitStr(row, ',');
		num = atoi(v[0].c_str());
		filename = v[1];
		jaccard = atof(v[2].c_str());
		if (!num)
			break;
	}
	return num;
}

int readFromCSV(string category, string csvname, vector<Validdata> &data)
{
	ifstream readin(csvname);
	string row, filename;
	int num;
	double jaccard;
	readin >> row;
	while (readin >> row) {
		vector<string> v = splitStr(row, ',');
		num = atoi(v[0].c_str());
		filename = v[1];
		jaccard = atof(v[2].c_str());
		if (!num)
			break;
		data.push_back(Validdata(num, filename, jaccard));
	}
	return num;
}

int readFromCSV(string category, string csvname, vector<string> &filenames)
{
	vector<Validdata> data;
	ifstream readin(csvname);
	string row, filename;
	int num;
	double jaccard;
	readin >> row;
	while (readin >> row) {
		vector<string> v = splitStr(row, ',');
		num = atoi(v[0].c_str());
		filename = v[1];
		jaccard = atof(v[2].c_str());
		if (!num)
			break;
		filenames.push_back(filename);
	}
	return num;
}

