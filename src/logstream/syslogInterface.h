//
// File: syslog.h
// Created by: Diogo Gomes <etdgomes@ua.pt>
// Created on: Sun Mar  9 15:12:23 2003
//

#ifndef _SYSLOGINTERFACE_H_
#define _SYSLOGINTERFACE_H_

#define OPTION 		LOG_CONS
#define FACILITY 	LOG_LOCAL0

#include <ostream>
#include <streambuf>
#include "logInterface.h"

extern "C" {
	#include <sys/types.h>
	#include <sys/stat.h>
	#include <fcntl.h>
}

namespace syslog {
	extern "C" {
		#include <sys/syslog.h>
	}
}
namespace logstream_helper {
	class syslogInterface : public logInterface
	{
		public:
			syslogInterface(int level);
			void setLevel(int level) { _level = level; };
			int level() { return _level; };
			int overflow(int c = EOF);
			void start_dump() {dump=true;};
			void stop_dump() {dump=false;};
		
		private:
			bool dump;
			int null;
			char *ptr;
			unsigned int len;
			int _level;
			char buf[1024];
	};
}

#endif	//_SYSLOGINTERFACE_H_
