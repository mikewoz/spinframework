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

#ifndef SPINLOGWX_H_
#define SPINLOGWX_H_

#include <iostream>
#include <fstream>
#include <time.h>
#include <stdio.h>

#include <wx/log.h>
#include <wx/string.h>

// code from http://www.advogato.org/person/elanthis/diary/363.html


enum LogPriority
{
    INFO, // regular unimportant log messages
    DEV, // debugging fluff
    ERROR, // it's dead, jim
};

class logbuf : public streambuf
{
    
public:
    
    // create a buffer and initialize our logfile
    logbuf(const char* logpath) :
        priority(INFO), buf(0), buflen(1024), bFILE(true), bCOUT(false)
    {
        // create our buffer
        buf = new char_type[buflen];
        setp(buf, buf + buflen);

        // open the log file
        logfile.open(logpath, ios::app);

        if (!logfile.is_open())
        {
            std::cout << "Error: Could not open log file: " << logpath << std::endl;
            exit(1);
        }

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
    bool bWXLG;

    
private:
    
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

        // now we stream the time, then the priority, then the message
        if (bCOUT) std::cout << shortTime << ' ';
        if (bFILE) logfile << longTime << ' ';

        switch (priority)
        {
        case INFO:
            if (bCOUT) std::cout << "[INFO:] ";
            if (bFILE) logfile << "[INFO:] ";
            break;
        case DEV:
            if (bCOUT) std::cout << "[DEBUG] ";
            if (bFILE) logfile << "[DEBUG] ";
            break;
        case ERROR:
            if (bCOUT) std::cout << "[ERROR] ";
            if (bFILE) logfile << "[ERROR] ";
            break;
        }


        if (bCOUT) std::cout.write(pbase(), pptr() - pbase());
        if (bFILE) logfile.write(pbase(), pptr() - pbase());
        
        if (bWXLG)
        {
            char wxBuf[1024];
            strncpy( wxBuf, pbase(), pptr()-pbase() );
            wxBuf[pptr()-pbase()-1] = '\0'; // remove the \n that comes from std::endl
            wxLogMessage(wxString(wxBuf,wxConvUTF8));
        }
        
        // flush output
        if (bCOUT) std::cout.flush();
        if (bFILE) logfile.flush();

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
    ofstream logfile;

    // current priority
    LogPriority priority;

    // our buffer
    char_type* buf;
    unsigned long buflen;
};


class spinLog : public std::ostream
{

public:
    // we initialize the ostream to use our logbuf
    spinLog(const char* logpath) : std::ostream(new logbuf(logpath))
    {
        buf = (logbuf*) rdbuf();
    }

    // set priority
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
    
    void enable_wxlog(bool b)
    {
        buf->bWXLG = b;
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


#endif
