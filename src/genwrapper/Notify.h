#ifndef NOTIFY_H_
#define NOTIFY_H_

#include <string>

#ifdef ERROR
#undef ERROR
#endif

struct Notify
{

    enum Level
    {
        QUIET    = 0,
        ERROR    = 1,
        WARNING  = 2,
        NOTICE   = 3,
        INFO     = 4,
        DEBUG    = 5
    };

    static void setLevel(Level level);

    static void error(const std::string &msg);
    static void warn(const std::string &msg);    
    static void notice(const std::string &msg);
    static void info(const std::string &msg);
    static void debug(const std::string &msg);

};

#endif
