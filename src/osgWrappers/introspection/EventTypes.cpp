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

#include <EventTypes.h>

// Must undefine IN and OUT macros defined in Windows headers
#ifdef IN
#undef IN
#endif
#ifdef OUT
#undef OUT
#endif

BEGIN_VALUE_REFLECTOR(spin::InfoMessage)
	I_DeclaringFile("EventTypes.h");
	I_Constructor1(IN, const std::string &, s,
	               Properties::NON_EXPLICIT,
	               ____InfoMessage__C5_std_string_R1,
	               "",
	               "");
	I_PublicMemberProperty(std::string, sceneID);
	I_PublicMemberProperty(std::string, serverAddr);
	I_PublicMemberProperty(int, serverUDPPort);
	I_PublicMemberProperty(int, serverTCPport);
	I_PublicMemberProperty(std::string, multicastAddr);
	I_PublicMemberProperty(int, multicastDataPort);
	I_PublicMemberProperty(int, multicastSyncPort);
	I_PublicMemberProperty(osg::Timer_t, lastUpdate);
END_REFLECTOR

