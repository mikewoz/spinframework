//
// File: syslog.h
// Created by: Diogo Gomes <etdgomes@ua.pt>
// Created on: Sun Mar  9 15:12:23 2003
//
#include "config.h"

#if HAVE_SYSLOG_H
#include "syslogInterface.h"
#include <unistd.h>
namespace logstream_helper { 
	syslogInterface::syslogInterface(int level) {
		ptr = buf;
		len = 0;
		_level = level;
		null = open("/dev/null", O_WRONLY);
		dump=false;
	}
	 
	int syslogInterface::overflow(int c) {
		if (c == '\n') {
			*ptr++ = '\n';
			*ptr = '\0';
			if(dump)
				write(null, buf, len + 1);
			else
				syslog::syslog(_level, "%s", buf);
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
#endif /* HAVE_SYSLOG_H */
