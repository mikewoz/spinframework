#include "FileSystemUtils.h"
#include "PathNameUtils.h"

#include <fstream>
#include <stdexcept>

#ifdef WIN32
#include <windows.h>
#else
#include <sys/stat.h>
#include <dirent.h>
#include <errno.h>
#endif

bool FileSystemUtils::createFile(const std::string &pathname, std::ofstream &ofs)
{
    std::string path(pathname);
    std::string::size_type p = 0;
    do
    {
        p = path.find_first_of("\\/", p+1);
        if (p != std::string::npos)
        {
            std::string dir(path.substr(0, p));

#ifdef WIN32

            HANDLE hf = CreateFile(dir.c_str(), 0, 0, 0, OPEN_EXISTING, FILE_FLAG_BACKUP_SEMANTICS, 0);
            if (hf == INVALID_HANDLE_VALUE)
            {
                if (!CreateDirectory(dir.c_str(), 0))
                    throw std::runtime_error("could not create directory `" + dir + "'");
            }
            else
                CloseHandle(hf);

#else
            if (mkdir(dir.c_str(), S_IRWXG | S_IRWXU) == -1)
            {
                if (errno != EEXIST)
                    throw std::runtime_error("could not create directori `" + dir + "'");
            }
#endif
                
        }
    } while (p != std::string::npos);

    ofs.open(pathname.c_str());
    return ofs.is_open();
}

void FileSystemUtils::findSourceFilesInDirectory(const std::string &dirname, std::set<std::string> &files)
{
#ifdef WIN32

    WIN32_FIND_DATA fd;    
    HANDLE hf = FindFirstFile((PathNameUtils::consolidateDelimiters(dirname, true) + "*.c*").c_str(), &fd);
    if (hf == INVALID_HANDLE_VALUE)
        return;

    do
    {
        files.insert(fd.cFileName);
    } while (FindNextFile(hf, &fd));

    FindClose(hf);

#else
    
    DIR* dir = opendir(dirname.c_str());
    if (!dir)
        return;
    
    struct dirent* de = readdir(dir);
    while (de)
    {
        std::string filename(de->d_name);
        std::string ext = PathNameUtils::getExtension(filename);
        if (!ext.empty() && ext[0] == 'c')
            files.insert(filename);
        de = readdir(dir);
    };
    closedir(dir);

#endif
}
