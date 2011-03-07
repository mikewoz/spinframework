//
// File: log.h
// Created by:  <>
// Created on: Sun Mar  9 15:36:49 2003
//

#ifndef _LOG_H_
#define _LOG_H_

#include "syslogInterface.h"
#include "stderrInterface.h"

namespace logstream {
    
    struct _setIdent {char *ident;};
    _setIdent setIdent(char *ident);

    struct _setDebugLevel {unsigned level;};
    _setDebugLevel setDebugLevel(unsigned level);

    struct _Level {unsigned level;};
    _Level level(unsigned level);
    
    class log : public std::ostream
    {
        public:
            log();
            log(char *,int level = LOG_ERR);
             ~log() {};
            friend log& operator<<(log &o, struct _setIdent _ident);
            friend log& operator<<(log &o, struct _setDebugLevel _level);
            friend log& operator<<(log &o, struct _Level _level);
            log &level(unsigned l) { *this << level(l); return *this; };
            
        private:
            logstream_helper::syslogInterface syslogbuf;
            logstream_helper::stderrInterface stderrbuf;
            logstream_helper::logInterface &buf;
            unsigned current_level;
        
    };
    
    static std::string endl("\n");
    static log::log lerr;
    static log::log lsyslog("");
}
#endif    //_LOG_H_
