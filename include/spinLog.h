// -----------------------------------------------------------------------------
// |    ___  ___  _  _ _     ___                                        _      |
// |   / __>| . \| || \ |   | __>_ _  ___ ._ _ _  ___  _ _ _  ___  _ _ | |__   |
// |   \__ \|  _/| ||   |   | _>| '_><_> || ' ' |/ ._>| | | |/ . \| '_>| / /   |
// |   <___/|_|  |_||_\_|   |_| |_|  <___||_|_|_|\___.|__/_/ \___/|_|  |_\_\   |
// |                                                                           |
// |---------------------------------------------------------------------------|
//
// http://spinframework.sourceforge.net
// Copyright (C) 2009 Mike Wozniewski, Zack Settel
//
// Developed/Maintained by:
//    Mike Wozniewski (http://www.mikewoz.com)
//    Zack Settel (http://www.sheefa.net/zack)
// 
// Principle Partners:
//    Shared Reality Lab, McGill University (http://www.cim.mcgill.ca/sre)
//    La Societe des Arts Technologiques (http://www.sat.qc.ca)
//
// Funding by:
//    NSERC/Canada Council for the Arts - New Media Initiative
//    Heritage Canada
//    Ministere du Developpement economique, de l'Innovation et de l'Exportation
//
// -----------------------------------------------------------------------------
//  This file is part of the SPIN Framework.
//
//  SPIN Framework is free software: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  SPIN Framework is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  along with SPIN Framework. If not, see <http://www.gnu.org/licenses/>.
// -----------------------------------------------------------------------------

#ifndef spinLog_H_
#define spinLog_H_

#include <iostream>
#include <fstream>
//#include <time.h>
#include <sys/time.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace spin
{

/**
 * Subtracts the `struct timeval' values X and Y, storing the result in RESULT.
 * Return 1 if the difference is negative, otherwise 0.
 */
static int timeval_subtract (struct timeval *result, struct timeval *x, struct timeval *y)
{
    // Perform the carry for the later subtraction by updating y:
    if (x->tv_usec < y->tv_usec) {
        int nsec = (y->tv_usec - x->tv_usec) / 1000000 + 1;
        y->tv_usec -= 1000000 * nsec;
        y->tv_sec += nsec;
    }
    if (x->tv_usec - y->tv_usec > 1000000) {
        int nsec = (y->tv_usec - x->tv_usec) / 1000000;
        y->tv_usec += 1000000 * nsec;
        y->tv_sec -= nsec;
    }

    // Compute the time remaining to wait. tv_usec is certainly positive:
    result->tv_sec = x->tv_sec - y->tv_sec;
    result->tv_usec = x->tv_usec - y->tv_usec;

    // Return 1 if result is negative:
    return x->tv_sec < y->tv_sec;
}

// code from http://www.advogato.org/person/elanthis/diary/363.html

enum LogPriority
{
    INFO, // regular unimportant log messages
    DEV, // debugging fluff
    ERROR, // it's dead, jim
};

/**
 * Buffer for text sent to the standard output so that it is stored to a file or displayed in a GUI.
 */
class logbuf : public std::streambuf
{
public:
    
    // create a buffer and initialize our logfile
    logbuf(const char* logpath) :
        bCOUT(false), bFILE(true), 
        logfile(),
        priority(INFO), buf(0), buflen(1024)
    {
        // create our buffer
        buf = new char_type[buflen];
        setp(buf, buf + buflen);

        // open the log file
        logfile.open(logpath, std::ios::app);

        if (! logfile.is_open())
        {
            std::cout << "Error: Could not open log file: " << logpath << std::endl;
            exit(1);
        }
        gettimeofday(&startTime, NULL);
    }

    // free our buffer
    ~logbuf()
    {
        sync();
        delete[] buf;
    }

    // set the priority to be used on the next call to sync()
    void set_priority(LogPriority p)
    {
        priority = p;
    }

    // logging modes:
    bool bCOUT;
    bool bFILE;
    
private:
    struct timeval startTime;
    
    // spit out the time, priority, and the log buffer to cerr and logfile
    int sync()
    {
        // nifty time formatting functions from the C standard library
        time_t t = time(NULL);
        tm* tmp = localtime(&t);
        char shortTime[128];
        char longTime[128];
        strftime(shortTime, sizeof(shortTime), "%H:%M:%S", tmp);
        strftime(longTime, sizeof(longTime), "%Y-%m-%d %H:%M:%S", tmp);
        
        // for more precise time:
        struct timeval elapsedTime;
        gettimeofday(&elapsedTime, NULL);
        struct timeval dt;
        timeval_subtract(&dt, &startTime, &elapsedTime);
        

        // now we stream the time, then the priority, then the message
        if (bCOUT)
            std::cout << shortTime << ' ';
        if (bFILE)
            logfile << longTime << ' ';
        
        logfile << (int)(-dt.tv_sec) << "." << (int)(dt.tv_usec) << ' ';

        /*
        switch (priority)
        {
        case INFO:
            if (bCOUT) cout << "[INFO:] ";
            if (bFILE) logfile << "[INFO:] ";
            break;
        case DEV:
            if (bCOUT) cout << "[DEBUG] ";
            if (bFILE) logfile << "[DEBUG] ";
            break;
        case ERROR:
            if (bCOUT) cout << "[ERROR] ";
            if (bFILE) logfile << "[ERROR] ";
            break;
        }
        */

        if (bCOUT)
            std::cout.write(pbase(), pptr() - pbase());
        if (bFILE)
            logfile.write(pbase(), pptr() - pbase());
        
        // flush output
        if (bCOUT)
            std::cout.flush();
        if (bFILE)
            logfile.flush();

        // reset our priority to INFO
        priority = INFO;

        // reset the buffer
        setp(pbase(), epptr());
        return 0;
    }

    // we ran out of space, so grow the buffer
    int overflow(int c)
    {
        // allocate a new buffer and copy our current data into it, then swap
        // it with the old buffer
        char_type newbuf[buflen + 1024];
        memcpy(newbuf, buf, buflen);
        delete[] buf;
        buf = newbuf;

        // now we need to stuff c into the buffer
        sputc(c);
        return 0;
    }

    // our log file
    std::ofstream logfile;

    // current priority
    LogPriority priority;

    // our buffer
    char_type* buf;
    unsigned long buflen;
};

/**
 * Redirects SPIN's standard output using a logbuf.
 */
class spinLog : public std::ostream
{
public:
    // we initialize the ostream to use our logbuf
    spinLog(const char* logpath) : std::ostream(new logbuf(logpath))
    {
        buf = (logbuf*) rdbuf();
    }

    /**
     * Sets priority
     */
    void set_priority(LogPriority pr)
    {
        buf->set_priority(pr);
    }
    
    void enable_cout(bool b)
    {
        buf->bCOUT = b;
    }
    
    void enable_logfile(bool b)
    {
        buf->bFILE = b;
    }
private:
    // our logbuf object
    logbuf *buf;
};

// set the priority for a spinLog/logbuf this must be a global function and not
// a member to work around C++'s type resolution of overloaded functions
static spinLog& operator<<(spinLog& vlog, LogPriority pr)
{
    vlog.set_priority(pr);
    return vlog;
}

} // end of namespace spin

#endif // not included

