//
// File: log.h
// Created by:  <>
// Created on: Sun Mar  9 15:36:49 2003
//

#include "log.h"
namespace logstream {
	log::log() : std::ostream(&stderrbuf),syslogbuf(LOG_ERR),stderrbuf(),buf(stderrbuf)
	{
		current_level=0;
	}
	
	log::log(char *ident,int level) : std::ostream(&syslogbuf),syslogbuf(level),stderrbuf(),buf(syslogbuf)
	{
		current_level=0;
		syslog::openlog(ident, OPTION, FACILITY);	
	}
	
	_setIdent setIdent(char *ident) 
	{
		_setIdent _ident;
		_ident.ident = ident;
		return _ident;
	};
	
	_setDebugLevel setDebugLevel(unsigned level)
	{
		_setDebugLevel _level;
		_level.level = level;
		return _level;
	};
	
	_Level level(unsigned level)
	{
		_Level _level;
		_level.level = level;
		return _level;
	};
	
	log::log &operator<<(log::log &o,struct _setIdent _ident) {
		syslog::closelog();
		syslog::openlog(_ident.ident, OPTION, FACILITY);	
		return o;
	}
	
	log::log &operator<<(log::log &o,struct _setDebugLevel _level) {
		o.current_level=_level.level;
		return o;
	}
	
	log::log &operator<<(log::log &o, struct _Level l) {
		if(l.level>o.current_level)
			o.buf.start_dump();
		else
			o.buf.stop_dump();
		return o;
	}
}
