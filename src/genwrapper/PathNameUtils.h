#ifndef PATHNAMEUTILS_H_
#define PATHNAMEUTILS_H_

#include <string>

struct PathNameUtils
{
    static std::string consolidateDelimiters(const std::string &pathname, bool add_trailer);
    static std::string getExtension(const std::string &pathname);
    static std::string stripExtension(const std::string &pathname);
    static std::string replaceExtension(const std::string &pathname, const std::string &newext);
    static std::string getDirectoryName(const std::string &pathname, bool add_trailer = true);
    static std::string getRelativePathAfter(const std::string &pathname, const std::string &delimiter);
};

#endif
