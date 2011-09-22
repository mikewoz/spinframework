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

#include <spinApp.h>
#include <spinBaseContext.h>
#include <spinUtil.h>

// Must undefine IN and OUT macros defined in Windows headers
#ifdef IN
#undef IN
#endif
#ifdef OUT
#undef OUT
#endif

BEGIN_OBJECT_REFLECTOR(spin::spinApp)
	I_DeclaringFile("spinApp.h");
	I_StaticMethod0(spin::spinApp &, Instance,
	                __spinApp_R1__Instance_S,
	                "",
	                "Meyers Singleton design patternFIXME: Do we really need this? ");
	I_Method1(void, setContext, IN, spin::spinBaseContext *, c,
	          Properties::NON_VIRTUAL,
	          __void__setContext__spinBaseContext_P1,
	          "",
	          "The first thing that any app must do is set the context for the spinApp singleton. This will provide the singleton with access to the liblo addresses so messages can be sent from anywhere. ");
	I_Method0(spin::spinBaseContext *, getContext,
	          Properties::NON_VIRTUAL,
	          __spinBaseContext_P1__getContext,
	          "",
	          "");
	I_Method0(void, createScene,
	          Properties::NON_VIRTUAL,
	          __void__createScene,
	          "",
	          "");
	I_Method0(void, destroyScene,
	          Properties::NON_VIRTUAL,
	          __void__destroyScene,
	          "",
	          "");
	I_Method0(void, registerUser,
	          Properties::NON_VIRTUAL,
	          __void__registerUser,
	          "",
	          "This method should be used to register a user for a listener-style SPIN client. The user is definitively created and stored in the current application context, even if a server is not running. ");
	I_Method3(void, spin::InfoMessage, IN, const std::string &, OSCpath, IN, const char *, types, IN, ..., x,
	          Properties::NON_VIRTUAL,
	          __void__InfoMessage__C5_std_string_R1__C5_char_P1__...,
	          "",
	          "This sends a variable length message.IMPORTANT: the list must be terminated with LO_ARGS_END, or this call will fail. This is used to do simple error checking on the sizes of parameters passed. ");
	I_Method3(void, spin::InfoMessage, IN, const std::string &, OSCpath, IN, const char *, types, IN, va_list, ap,
	          Properties::NON_VIRTUAL,
	          __void__InfoMessage__C5_std_string_R1__C5_char_P1__va_list,
	          "",
	          "");
	I_Method2(void, spin::InfoMessage, IN, const std::string &, OSCpath, IN, lo_message, msg,
	          Properties::NON_VIRTUAL,
	          __void__InfoMessage__C5_std_string_R1__lo_message,
	          "",
	          "");
	I_Method2(void, SceneMessage, IN, const char *, types, IN, ..., x,
	          Properties::NON_VIRTUAL,
	          __void__SceneMessage__C5_char_P1__...,
	          "",
	          "");
	I_Method2(void, SceneMessage, IN, const char *, types, IN, va_list, ap,
	          Properties::NON_VIRTUAL,
	          __void__SceneMessage__C5_char_P1__va_list,
	          "",
	          "");
	I_Method1(void, SceneMessage, IN, lo_message, msg,
	          Properties::NON_VIRTUAL,
	          __void__SceneMessage__lo_message,
	          "",
	          "");
	I_Method3(void, NodeMessage, IN, const char *, nodeId, IN, const char *, types, IN, ..., x,
	          Properties::NON_VIRTUAL,
	          __void__NodeMessage__C5_char_P1__C5_char_P1__...,
	          "",
	          "");
	I_Method3(void, NodeMessage, IN, const char *, nodeId, IN, const char *, types, IN, va_list, ap,
	          Properties::NON_VIRTUAL,
	          __void__NodeMessage__C5_char_P1__C5_char_P1__va_list,
	          "",
	          "");
	I_Method2(void, NodeMessage, IN, const char *, nodeId, IN, lo_message, msg,
	          Properties::NON_VIRTUAL,
	          __void__NodeMessage__C5_char_P1__lo_message,
	          "",
	          "");
	I_MethodWithDefaults3(void, NodeBundle, IN, spin::t_symbol *, nodeSym, , IN, std::vector< lo_message >, msgs, , IN, lo_address, addr, 0,
	                      Properties::NON_VIRTUAL,
	                      __void__NodeBundle__t_symbol_P1__std_vectorT1_lo_message___lo_address,
	                      "",
	                      "");
	I_MethodWithDefaults2(void, SceneBundle, IN, std::vector< lo_message >, msgs, , IN, lo_address, addr, 0,
	                      Properties::NON_VIRTUAL,
	                      __void__SceneBundle__std_vectorT1_lo_message___lo_address,
	                      "",
	                      "");
	I_Method1(void, setSceneID, IN, const std::string &, s,
	          Properties::NON_VIRTUAL,
	          __void__setSceneID__C5_std_string_R1,
	          "",
	          "");
	I_Method0(std::string, getSceneID,
	          Properties::NON_VIRTUAL,
	          __std_string__getSceneID,
	          "",
	          "");
	I_Method1(void, setSyncStart, IN, osg::Timer_t, t,
	          Properties::NON_VIRTUAL,
	          __void__setSyncStart__osg_Timer_t,
	          "",
	          "");
	I_Method0(osg::Timer_t, getSyncStart,
	          Properties::NON_VIRTUAL,
	          __osg_Timer_t__getSyncStart,
	          "",
	          "");
	I_Method0(bool, initPython,
	          Properties::NON_VIRTUAL,
	          __bool__initPython,
	          "",
	          "initializes the embedded python interpreter ");
	I_Method1(bool, execPython, IN, const std::string &, cmd,
	          Properties::NON_VIRTUAL,
	          __bool__execPython__C5_std_string_R1,
	          "",
	          "Runs some Python code and returns its success or not. ");
	I_Method0(std::string, getCurrentPyException,
	          Properties::NON_VIRTUAL,
	          __std_string__getCurrentPyException,
	          "",
	          "Returns a string containing the most recent Python exception, if any. ");
	I_Method1(void, setUserID, IN, const std::string &, id,
	          Properties::NON_VIRTUAL,
	          __void__setUserID__C5_std_string_R1,
	          "",
	          "");
	I_Method0(std::string, getUserID,
	          Properties::NON_VIRTUAL,
	          __std_string__getUserID,
	          "",
	          "");
	I_SimpleProperty(spin::spinBaseContext *, Context, 
	                 __spinBaseContext_P1__getContext, 
	                 __void__setContext__spinBaseContext_P1);
	I_SimpleProperty(std::string, CurrentPyException, 
	                 __std_string__getCurrentPyException, 
	                 0);
	I_SimpleProperty(std::string, SceneID, 
	                 __std_string__getSceneID, 
	                 __void__setSceneID__C5_std_string_R1);
	I_SimpleProperty(osg::Timer_t, SyncStart, 
	                 __osg_Timer_t__getSyncStart, 
	                 __void__setSyncStart__osg_Timer_t);
	I_SimpleProperty(std::string, UserID, 
	                 __std_string__getUserID, 
	                 __void__setUserID__C5_std_string_R1);
	I_PublicMemberProperty(boost::python::object, _pyMainModule);
	I_PublicMemberProperty(boost::python::object, _pyNamespace);
	I_PublicMemberProperty(bool, _pyInitialized);
	I_PublicMemberProperty(osg::ref_ptr< spin::UserNode >, userNode);
	I_PublicMemberProperty(bool, hasAudioRenderer);
	I_PublicMemberProperty(spin::SceneManager *, sceneManager);
	I_PublicMemberProperty(spin::MediaManager *, mediaManager);
END_REFLECTOR

