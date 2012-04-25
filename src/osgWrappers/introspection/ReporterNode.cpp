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

#include <ReporterNode.h>
#include <SceneManager.h>

// Must undefine IN and OUT macros defined in Windows headers
#ifdef IN
#undef IN
#endif
#ifdef OUT
#undef OUT
#endif

BEGIN_VALUE_REFLECTOR(spin::_reporterTarget)
	I_DeclaringFile("ReporterNode.h");
	I_Constructor0(_____reporterTarget,
	               "",
	               "");
	I_PublicMemberProperty(osg::observer_ptr< spin::ReferencedNode >, node);
	I_PublicMemberProperty(osg::Matrix, matrix);
	I_PublicMemberProperty(bool, contained);
END_REFLECTOR

BEGIN_OBJECT_REFLECTOR(spin::ReporterNode)
	I_DeclaringFile("ReporterNode.h");
	I_BaseType(spin::ReferencedNode);
	I_Constructor2(IN, spin::SceneManager *, sceneManager, IN, char *, initID,
	               ____ReporterNode__SceneManager_P1__char_P1,
	               "",
	               "");
	I_Method0(void, debug,
	          Properties::VIRTUAL,
	          __void__debug,
	          "",
	          "Debug print (to log/console) ");
	I_Method0(void, callbackUpdate,
	          Properties::VIRTUAL,
	          __void__callbackUpdate,
	          "",
	          "The update callback for ReporterNode checks to see if a target or the the ReporterNode's global matrix has changed (ie, whether it has been moved or not). If so, it updates the internal matrices, and calls sendReports() ");
	I_Method1(void, sendReports, IN, spin::reporterTarget *, target,
	          Properties::NON_VIRTUAL,
	          __void__sendReports__reporterTarget_P1,
	          "",
	          "sendReports checks which reportTypes are enabled, and actually performs computation for necessary reports, which are then sent out on the network ");
	I_Method1(void, addTarget, IN, const char *, targetID,
	          Properties::NON_VIRTUAL,
	          __void__addTarget__C5_char_P1,
	          "",
	          "Add a target node to the report list ");
	I_Method1(void, removeTarget, IN, const char *, targetID,
	          Properties::NON_VIRTUAL,
	          __void__removeTarget__C5_char_P1,
	          "",
	          "Remove a target from the report list ");
	I_Method2(void, setReporting, IN, const char *, type, IN, bool, enabled,
	          Properties::NON_VIRTUAL,
	          __void__setReporting__C5_char_P1__bool,
	          "",
	          "This enables or disables a particular reporting type ");
	I_Method1(int, getReporting, IN, const char *, type,
	          Properties::NON_VIRTUAL,
	          __int__getReporting__C5_char_P1,
	          "",
	          "This returns the reporting type that is currently set. ");
	I_Method1(void, setMaxRate, IN, float, hz,
	          Properties::NON_VIRTUAL,
	          __void__setMaxRate__float,
	          "",
	          "Set the maximum reporting rate (hz). Note: updates are only sent when necessary, so there is no constant reporting mode. ");
	I_Method0(float, getMaxRate,
	          Properties::NON_VIRTUAL,
	          __float__getMaxRate,
	          "",
	          "This returns the current set reporting rate in Hz. ");
	I_Method0(std::vector< lo_message >, getState,
	          Properties::VIRTUAL,
	          __std_vectorT1_lo_message___getState,
	          "",
	          "For each subclass of ReferencedNode, we override the getState() method to fill the vector with the correct set of methods for this particular node ");
	I_SimpleProperty(float, MaxRate, 
	                 __float__getMaxRate, 
	                 __void__setMaxRate__float);
	I_SimpleProperty(std::vector< lo_message >, State, 
	                 __std_vectorT1_lo_message___getState, 
	                 0);
END_REFLECTOR

TYPE_NAME_ALIAS(struct spin::_reporterTarget, spin::reporterTarget)

