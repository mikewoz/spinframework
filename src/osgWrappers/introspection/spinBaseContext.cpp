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

#include <EventHandler.h>
#include <spinBaseContext.h>

// Must undefine IN and OUT macros defined in Windows headers
#ifdef IN
#undef IN
#endif
#ifdef OUT
#undef OUT
#endif

BEGIN_ENUM_REFLECTOR(spin::spinBaseContext::SpinContextMode)
	I_DeclaringFile("spinBaseContext.h");
	I_EnumLabel(spin::spinBaseContext::SERVER_MODE);
	I_EnumLabel(spin::spinBaseContext::CLIENT_MODE);
END_REFLECTOR

BEGIN_ABSTRACT_OBJECT_REFLECTOR(spin::spinBaseContext)
	I_DeclaringFile("spinBaseContext.h");
	I_Constructor0(____spinBaseContext,
	               "",
	               "Constructor.This is where the actual default port numbers and multicast groups are defined. ");
	I_Method0(bool, isServer,
	          Properties::NON_VIRTUAL,
	          __bool__isServer,
	          "",
	          "");
	I_Method0(bool, start,
	          Properties::PURE_VIRTUAL,
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
	I_Method1(bool, startThread, IN, void *(*)(void *), threadFunction,
	          Properties::NON_VIRTUAL,
	          __bool__startThread__void_P1(P1)(void_P1),
	          "",
	          "Starts the context thread (passed as *threadFunction from a derived class)Startup point of the server's thread.Startup point of the server's thread. ");
	I_Method0(void, stop,
	          Properties::NON_VIRTUAL,
	          __void__stop,
	          "",
	          "Stops the currently running thread ");
	I_Method0(bool, isRunning,
	          Properties::NON_VIRTUAL,
	          __bool__isRunning,
	          "",
	          "");
	I_Method1(void, addInfoHandler, IN, spin::EventHandler *, obs,
	          Properties::NON_VIRTUAL,
	          __void__addInfoHandler__EventHandler_P1,
	          "",
	          "Add an event handler to the list of classes that will be notified when a message is received on the INFO channel: ");
	I_Method1(void, removeInfoHandler, IN, spin::EventHandler *, obs,
	          Properties::NON_VIRTUAL,
	          __void__removeInfoHandler__EventHandler_P1,
	          "",
	          "Remove an INFO channel event handler ");
	I_Method1(void, removeHandlerForAllEvents, IN, spin::EventHandler *, obs,
	          Properties::NON_VIRTUAL,
	          __void__removeHandlerForAllEvents__EventHandler_P1,
	          "",
	          "");
	I_Method0(bool, canAutoAssignPorts,
	          Properties::NON_VIRTUAL,
	          __bool__canAutoAssignPorts,
	          "",
	          "Check if spin is allowed to assign automatic ports (eg, in the case where a port is busy). This is usually true, but if a user specifies ports manually with command-line options, this becomes false. ");
	I_Method1(void, setTTL, IN, int, ttl,
	          Properties::NON_VIRTUAL,
	          __void__setTTL__int,
	          "",
	          "Set the time-to-live for multicast packets (corresponds to the number of routers a packet will hop). ");
	I_Method1(void, setSecureBroadcast, IN, bool, b,
	          Properties::VIRTUAL,
	          __void__setSecureBroadcast__bool,
	          "",
	          "Enabling secure broadcast means that TCP subscribers will be notified by TCP of EVERY node and scene update. This is in addition to the regular multicast, so subscribers who also listen to multicast will receive duplicate messages. In the case of spinviewer, we stop polling UDP receivers when this flag is set (see pollUpdates() in spinClientContext). ");
	I_Method0(bool, hasSecureBroadcast,
	          Properties::NON_VIRTUAL,
	          __bool__hasSecureBroadcast,
	          "",
	          "");
	I_StaticMethod1(void, sigHandler, IN, int, signum,
	                __void__sigHandler__int_S,
	                "",
	                "");
	I_StaticMethod6(int, connectionCallback, IN, const char *, path, IN, const char *, types, IN, lo_arg **, argv, IN, int, argc, IN, void *, data, IN, void *, user_data,
	                __int__connectionCallback__C5_char_P1__C5_char_P1__lo_arg_P1P1__int__void_P1__void_P1_S,
	                "",
	                "");
	I_StaticMethod6(int, nodeCallback, IN, const char *, path, IN, const char *, types, IN, lo_arg **, argv, IN, int, argc, IN, void *, data, IN, void *, user_data,
	                __int__nodeCallback__C5_char_P1__C5_char_P1__lo_arg_P1P1__int__void_P1__void_P1_S,
	                "",
	                "Callback for messages sent to a node in the scene graph.Messages to node should have an OSC address in the form /SPIN/<scene id>=\"\">/<node id>=\"\"> Their first argument is the name of the method to call.Methods to manage Python scripts for a node: addCronScript <label> <path> <frequency>addEventScript <label> <event> <path> [*args...]enableCronScript <label>removeCronScript <label>enableEventScript <label>removeEventScript <label> We use C++ introspection to figure out the other methods that can be called for a given node. ");
	I_StaticMethod6(int, sceneCallback, IN, const char *, path, IN, const char *, types, IN, lo_arg **, argv, IN, int, argc, IN, void *, data, IN, void *, user_data,
	                __int__sceneCallback__C5_char_P1__C5_char_P1__lo_arg_P1P1__int__void_P1__void_P1_S,
	                "",
	                "Callback for the OSC message to the whole scene.The address of the OSC messages sent to the scene are in the form /SPIN/<scene id>=\"\"> <method name>=\"\"> [args...]They are used mostly to delete all nodes from a scene, or to ask the server to refresh the information about all nodes. It's also possible to save the current scene graph to an XML file, and to load a previously saved XML file.Some valid method include: clearclearUsersclearStatesuserRefreshrefreshrefreshSubscribersgetNodeListnodeList [node names...] : Creates many nodesstateList [] : Creates many state setsexportScene [] []load [XML file]save [XML file]saveAll [XML file]saveUsers [XML file]createNode [node name] [node type]createStateSet [name] [type]deleteNode [name]deleteGraph [name]  ");
	I_StaticMethod6(int, logCallback, IN, const char *, path, IN, const char *, types, IN, lo_arg **, argv, IN, int, argc, IN, void *, data, IN, void *, user_data,
	                __int__logCallback__C5_char_P1__C5_char_P1__lo_arg_P1P1__int__void_P1__void_P1_S,
	                "",
	                "");
	I_StaticMethod6(int, debugCallback, IN, const char *, path, IN, const char *, types, IN, lo_arg **, argv, IN, int, argc, IN, void *, data, IN, void *, user_data,
	                __int__debugCallback__C5_char_P1__C5_char_P1__lo_arg_P1P1__int__void_P1__void_P1_S,
	                "",
	                "");
	I_StaticMethod3(void, oscParser_error, IN, int, num, IN, const char *, msg, IN, const char *, path,
	                __void__oscParser_error__int__C5_char_P1__C5_char_P1_S,
	                "",
	                "");
	I_ProtectedMethod1(void, setLog, IN, spin::spinLog &, log,
	                   Properties::NON_VIRTUAL,
	                   Properties::NON_CONST,
	                   __void__setLog__spinLog_R1,
	                   "",
	                   "");
	I_ProtectedMethod0(void, createServers,
	                   Properties::PURE_VIRTUAL,
	                   Properties::NON_CONST,
	                   __void__createServers,
	                   "",
	                   "this method is used by both spinClientContext and spinServerContext ");
	I_SimpleProperty(bool, SecureBroadcast, 
	                 0, 
	                 __void__setSecureBroadcast__bool);
	I_SimpleProperty(int, TTL, 
	                 0, 
	                 __void__setTTL__int);
	I_PublicMemberProperty(std::vector< lo_address >, lo_txAddrs_);
	I_PublicMemberProperty(std::vector< lo_address >, lo_rxAddrs_);
	I_PublicMemberProperty(std::vector< lo_server >, lo_rxServs_);
	I_PublicMemberProperty(lo_address, lo_infoAddr);
	I_PublicMemberProperty(lo_address, lo_syncAddr);
	I_PublicMemberProperty(lo_server, lo_infoServ_);
	I_PublicMemberProperty(std::string, tcpPort_);
	I_PublicMemberProperty(lo_server, lo_tcpRxServer_);
	I_PublicMemberProperty(bool, doDiscovery_);
END_REFLECTOR

