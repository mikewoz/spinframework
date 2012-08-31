// ***************************************************************************
//
//   Generated automatically by genwrapper.
//   Please DO NOT EDIT this file!
//
// ***************************************************************************

#include <cppintrospection/ReflectionMacros>
#include <cppintrospection/TypedMethodInfo>
#include <cppintrospection/StaticMethodInfo>
#include <cppintrospection/Attributes>

#include <spinServerContext.h>

// Must undefine IN and OUT macros defined in Windows headers
#ifdef IN
#undef IN
#endif
#ifdef OUT
#undef OUT
#endif

BEGIN_OBJECT_REFLECTOR(spin::spinServerContext)
	I_DeclaringFile("spinServerContext.h");
	I_BaseType(spin::spinBaseContext);
	I_Constructor0(____spinServerContext,
	               "",
	               "");
	I_Method0(bool, start,
	          Properties::VIRTUAL,
	          __bool__start,
	          "",
	          "");
	I_Method0(void, debugPrint,
	          Properties::VIRTUAL,
	          __void__debugPrint,
	          "",
	          "");
	I_Method1(void, addCommandLineOptions, IN, osg::ArgumentParser *, arguments,
	          Properties::VIRTUAL,
	          __void__addCommandLineOptions__osg_ArgumentParser_P1,
	          "",
	          "");
	I_Method1(int, parseCommandLineOptions, IN, osg::ArgumentParser *, arguments,
	          Properties::VIRTUAL,
	          __int__parseCommandLineOptions__osg_ArgumentParser_P1,
	          "",
	          "");
	I_Method0(void, refreshSubscribers,
	          Properties::NON_VIRTUAL,
	          __void__refreshSubscribers,
	          "",
	          "");
	I_Method1(void, setSecureBroadcast, IN, bool, b,
	          Properties::VIRTUAL,
	          __void__setSecureBroadcast__bool,
	          "",
	          "Enabling secure broadcast means that TCP subscribers will be notified by TCP of EVERY node and scene update. This is in addition to the regular multicast, so subscribers who also listen to multicast will receive duplicate messages. In the case of spinviewer, we stop polling UDP receivers when this flag is set (see pollUpdates() in spinClientContext). ");
	I_Method0(void, startSyncThread,
	          Properties::NON_VIRTUAL,
	          __void__startSyncThread,
	          "",
	          "Starts the thread that sends synchronization timecode (syncThread) ");
	I_Method0(bool, shouldAutoClean,
	          Properties::NON_VIRTUAL,
	          __bool__shouldAutoClean,
	          "",
	          "A flag that decides if user nodes should be automatically cleaned up (ie, their entire subgraph deleted) if they stop sending ping messages. This is set by the disable-auto-cleanup argument. ");
	I_Method2(bool, applyHTTPMessage, IN, std::string, path, IN, const Poco::Net::HTMLForm &, form,
	          Properties::NON_VIRTUAL,
	          __bool__applyHTTPMessage__std_string__C5_Poco_Net_HTMLForm_R1,
	          "",
	          "");
	I_Method0(unsigned short, getHttpPort,
	          Properties::NON_VIRTUAL,
	          __unsigned_short__getHttpPort,
	          "",
	          "");
	I_SimpleProperty(unsigned short, HttpPort, 
	                 __unsigned_short__getHttpPort, 
	                 0);
	I_SimpleProperty(bool, SecureBroadcast, 
	                 0, 
	                 __void__setSecureBroadcast__bool);
	I_PublicMemberProperty(std::map< std::string COMMA  lo_address >, tcpClientAddrs_);
END_REFLECTOR

