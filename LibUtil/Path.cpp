/*
 * Paths.cpp
 *
 *  Created on: 29.11.2010
 *      Author: dube
 */

#include <sys/types.h>
#include <dirent.h>
#include <iostream>
#include <sstream>
#include <fstream>

#include "Path.h"
#include "RamaxxException.h"

using namespace std;

Path::Path(string path)
{
	Path::splitPath(path);
}

Path::~Path()
{
}

string Path::getExtension() {
	return this->mExtension;
}

string Path::getFilename() {
	return this->mFilename + '.' + this->mExtension;
}

string Path::getFilenameWoExtension() {
	return this->mFilename;
}

string Path::getDirectory() {
	return this->mDirectory;
}

string Path::getPath() {
	return this->mDirectory + "/" + this->getFilename();
}

void Path::splitPath(string path) {
	this->mDirectory = "";
	this->mFilename = "";
	this->mExtension = "";
	pair<string, string> p = rSplitString(path, "/");
	this->mDirectory = p.first;
	p = rSplitString(p.second, ".");
	this->mFilename = p.first;
	this->mExtension = p.second;
}

pair<string, string> Path::rSplitString(string path, string seperator) {
        string::size_type i = path.rfind(seperator);
        return SplitStringAtPos(path, i);
}

pair<string, string> Path::lSplitString(string path, string seperator) {
        string::size_type i = path.find(seperator);
        return SplitStringAtPos(path, i);
}

pair<string, string> Path::SplitStringAtPos(string path,
        string::size_type pos) {
        string left = "";
        string right = "";
        if (pos == string::npos)
                right = path;
        else if (pos + 1 >= path.length())
                left = path;
        else {
                left = path.substr(0, pos);
                right = path.substr(pos + 1);
        }
        return pair<string, string>(left, right);
}

vector<string> Path::getFiles(string directory) {
	DIR* dir;
	struct dirent *entry;
	if (!(dir = opendir(directory.c_str())))
		throw RamaxxException("Could not open directory '" + directory
				+	"'.");
	vector<string> result;
	//Iterato over the files
	while ((entry = readdir(dir)))
		result.push_back(directory + '/' + entry->d_name);
	closedir(dir);
	return result;
}

string Path::getExtension_(string path) {
	string extension = "";
	string::size_type point = path.rfind(".");

	if (point != string::npos)
		extension = path.substr(point + 1);
	return extension;
}

string Path::createUniqueFilename(string filename) {
	bool gotIt = false;
	int index = 0;
	string file;
	Path p = Path(filename);

	while (not gotIt) {
		ostringstream s;
		s << p.getDirectory() << "/" << p.getFilenameWoExtension() << "." <<
				index << "." << p.getExtension();
		file = s.str();
		gotIt = not fileExists(file);
		index ++;
	}
	return file;
}

bool Path::fileExists(const string filename) {
	ifstream file(filename.c_str());
	if (file)
		return true;
	return false;
}
