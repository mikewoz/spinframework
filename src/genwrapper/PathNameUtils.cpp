#include "PathNameUtils.h"

#include <string>

std::string PathNameUtils::getExtension(const std::string &pathname)
{
    std::string::size_type p = pathname.rfind('.');
    if (p == std::string::npos) 
        return "";
    return pathname.substr(p+1, std::string::npos);
}

std::string PathNameUtils::consolidateDelimiters(const std::string &pathname, bool add_trailer)
{
    std::string res(pathname);
    
    for (std::string::iterator i=res.begin(); i!=res.end(); ++i)
        if (*i == '\\') *i = '/';

    if (add_trailer)
    {
        if (!res.empty() && *res.rbegin() != '/')
            res.push_back('/');
    }

    return res;
}

std::string PathNameUtils::stripExtension(const std::string &pathname)
{
    std::string::size_type p = pathname.rfind('.');
    if (p == std::string::npos) 
        return pathname;
    return pathname.substr(0, p);
}

std::string PathNameUtils::replaceExtension(const std::string &pathname, const std::string &newext)
{
    return stripExtension(pathname) + "." + newext;
}

std::string PathNameUtils::getDirectoryName(const std::string &pathname, bool add_trailer)
{
    std::string::size_type p = pathname.find_last_of("\\/");
    if (p == std::string::npos)
        return add_trailer ? "./" : ".";
    return pathname.substr(0, p) + (add_trailer ? "/" : "");
}

std::string PathNameUtils::getRelativePathAfter(const std::string &pathname, const std::string &delimiter)
{
    std::string::size_type p = pathname.rfind(delimiter);
    if (p == std::string::npos)
        return pathname;
    return pathname.substr(p + delimiter.length());
}
