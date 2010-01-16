//
// File: syslog.h
// Created by: Diogo Gomes <etdgomes@ua.pt>
// Created on: Sun Mar  9 15:12:23 2003
//

#include "stderrInterface.h"
#include <unistd.h>
namespace logstream_helper {
	stderrInterface::stderrInterface() {
		ptr = buf;
		len = 0;
		null = open("/dev/null", O_WRONLY);
		dump=false;
	}
	 
	int stderrInterface::overflow(int c) {
		if (c == '\n') {
			*ptr++ = '\n';
			*ptr = '\0';
			if(dump)
				write(null, buf, len + 1);
			else
				write(2, buf, len + 1);
			ptr = buf;
			len = 0;
			return 0;
		}
		if ((len + 2) >= sizeof(buf))
			return EOF;
		*ptr++ = c;
		len++;
		return 0;
	}
}
