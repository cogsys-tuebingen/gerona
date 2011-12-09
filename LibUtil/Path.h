/*
 * Paths.h
 *
 *  Created on: 29.11.2010
 *      Author: dube
 */

#ifndef PATH_H_
#define PATH_H_

#include <string>
#include <vector>

class Path
{
public:
	Path(std::string path);
	virtual ~Path();

	static std::vector<std::string> getFiles(std::string directory);
	static std::string getExtension_(std::string path);
	static bool fileExists(const std::string filename);
	static std::string createUniqueFilename(std::string filename);
	std::string getFilename();
	std::string getFilenameWoExtension();
	std::string getExtension();
	std::string getDirectory();
	std::string getPath();

private:
	std::pair<std::string, std::string> rSplitString(std::string path,
			std::string seperator);
	std::pair<std::string, std::string> SplitStringAtPos(
                std::string path, std::string::size_type pos);
	std::pair<std::string, std::string> lSplitString(std::string path,
                std::string seperator);
	void splitPath(std::string path);

	std::string mDirectory;
	std::string mFilename;
	std::string mExtension;
};

#endif /* PATH_H_ */
