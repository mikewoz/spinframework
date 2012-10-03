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

#include <spinClientContext.h>

// Must undefine IN and OUT macros defined in Windows headers
#ifdef IN
#undef IN
#endif
#ifdef OUT
#undef OUT
#endif

BEGIN_OBJECT_REFLECTOR(spin::spinClientContext)
	I_DeclaringFile("spinClientContext.h");
	I_BaseType(spin::spinBaseContext);
	I_Constructor0(____spinClientContext,
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
	I_Method0(int, pollUpdates,
	          Properties::NON_VIRTUAL,
	          __int__pollUpdates,
	          "",
	          "");
	I_Method1(void, setReliableBroadcast, IN, bool, b,
	          Properties::VIRTUAL,
	          __void__setReliableBroadcast__bool,
	          "",
	          "Reliable broadcast means that TCP subscribers will be notified by TCP of EVERY node and scene update. This is in addition to the regular multicast, so subscribers who also listen to multicast will receive duplicate messages. In the case of spinviewer, we stop polling UDP receivers when this flag is set (see pollUpdates() in spinClientContext). ");
	I_Method0(void, subscribe,
	          Properties::NON_VIRTUAL,
	          __void__subscribe,
	          "",
	          "Register the client's ip and port for reliable communication with the server ");
	I_SimpleProperty(bool, ReliableBroadcast, 
	                 0, 
	                 __void__setReliableBroadcast__bool);
	I_PublicMemberProperty(lo_server, lo_syncServ);
END_REFLECTOR

