#ifndef FILESYSTEMUTILS_H_
#define FILESYSTEMUTILS_H_

#include <string>
#include <iosfwd>
#include <set>

struct FileSystemUtils
{
    static bool createFile(const std::string &pathname, std::ofstream &ofs);
    static void findSourceFilesInDirectory(const std::string &dirname, std::set<std::string> &files);
};

#endif
