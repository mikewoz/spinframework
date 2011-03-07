//
// File: logInterface.h
// Created by: Diogo Gomes <etdgomes@ua.pt>
// Created on: Sun Mar  9 16:20:17 2003
//

#ifndef _LOGINTERFACE_H_
#define _LOGINTERFACE_H_

#include <ostream>
namespace logstream_helper {
    class logInterface : public std::streambuf
    {
        public:
            virtual int overflow(int c = EOF) = 0;    
            virtual void start_dump() =0;
            virtual void stop_dump() =0;
    };
}

#endif    //_LOGINTERFACE_H_
