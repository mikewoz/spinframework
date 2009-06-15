#include "Notify.h"

#include <stdexcept>
#include <iostream>

namespace
{
    Notify::Level level__ = Notify::NOTICE;
}

void Notify::setLevel(Level level)
{
    level__ = level;
}

void Notify::warn(const std::string &msg)
{
    if (level__ >= WARNING)
        std::cerr << "* WARNING: " << msg << std::endl;
}

void Notify::error(const std::string &msg)
{
    if (level__ >= ERROR)
        throw std::runtime_error("* ERROR: " + msg);
    else
        throw std::runtime_error("");
}

void Notify::notice(const std::string &msg)
{
    if (level__ >= NOTICE)
        std::clog << msg << std::endl;
}


void Notify::info(const std::string &msg)
{
    if (level__ >= INFO)
        std::cout << "* INFO: " << msg << "\n";
}

void Notify::debug(const std::string &msg)
{
    if (level__ >= DEBUG)
        std::cout << "* DEBUG: " << msg << "\n";
}
