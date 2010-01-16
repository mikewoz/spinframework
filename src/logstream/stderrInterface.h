//
// File: syslog.h
// Created by: Diogo Gomes <etdgomes@ua.pt>
// Created on: Sun Mar  9 15:12:23 2003
//

#ifndef _STDERRINTERFACE_H_
#define _STDERRINTERFACE_H_

#include <ostream>
#include <streambuf>
#include "logInterface.h"

extern "C" {
	#include <sys/types.h>
	#include <sys/stat.h>
	#include <fcntl.h>
}

namespace logstream_helper {
	class stderrInterface : public logInterface
	{
		public:
			stderrInterface();
			int overflow(int c = EOF);
			void start_dump() {dump=true;};
			void stop_dump() {dump=false;};
		
		private:
			bool dump;
			int null;
			char *ptr;
			unsigned int len;
			char buf[1024];
	};
}

#endif	//_STDERRINTERFACE_H_
