// ***************************************************************************
//
//   Generated automatically by genwrapper.
//   Please DO NOT EDIT this file!
//
// ***************************************************************************

#include <osgIntrospection/ReflectionMacros>
#include <osgIntrospection/TypedMethodInfo>
#include <osgIntrospection/StaticMethodInfo>
#include <osgIntrospection/Attributes>

#include <DSPNode.h>
#include <SceneManager.h>
#include <SoundConnection.h>

// Must undefine IN and OUT macros defined in Windows headers
#ifdef IN
#undef IN
#endif
#ifdef OUT
#undef OUT
#endif

BEGIN_OBJECT_REFLECTOR(DSPNode)
	I_DeclaringFile("DSPNode.h");
	I_BaseType(GroupNode);
	I_Constructor2(IN, SceneManager *, sceneManager, IN, char *, initID,
	               ____DSPNode__SceneManager_P1__char_P1,
	               "",
	               "");
	I_Method1(SoundConnection *, getConnection, IN, DSPNode *, snk,
	          Properties::NON_VIRTUAL,
	          __SoundConnection_P1__getConnection__DSPNode_P1,
	          "",
	          "");
	I_Method1(SoundConnection *, getConnection, IN, const char *, snk,
	          Properties::NON_VIRTUAL,
	          __SoundConnection_P1__getConnection__C5_char_P1,
	          "",
	          "");
	I_Method1(void, connect, IN, DSPNode *, snk,
	          Properties::NON_VIRTUAL,
	          __void__connect__DSPNode_P1,
	          "",
	          "");
	I_Method1(void, connect, IN, const char *, snk,
	          Properties::NON_VIRTUAL,
	          __void__connect__C5_char_P1,
	          "",
	          "");
	I_Method1(void, connectSource, IN, const char *, src,
	          Properties::NON_VIRTUAL,
	          __void__connectSource__C5_char_P1,
	          "",
	          "");
	I_Method1(void, disconnect, IN, const char *, snk,
	          Properties::NON_VIRTUAL,
	          __void__disconnect__C5_char_P1,
	          "",
	          "");
	I_Method1(void, setActive, IN, int, i,
	          Properties::NON_VIRTUAL,
	          __void__setActive__int,
	          "",
	          "");
	I_Method1(void, setPlugin, IN, const char *, filename,
	          Properties::NON_VIRTUAL,
	          __void__setPlugin__C5_char_P1,
	          "",
	          "");
	I_Method0(int, getActive,
	          Properties::NON_VIRTUAL,
	          __int__getActive,
	          "",
	          "");
	I_Method0(const char *, getPlugin,
	          Properties::NON_VIRTUAL,
	          __C5_char_P1__getPlugin,
	          "",
	          "");
	I_Method0(std::vector< lo_message >, getState,
	          Properties::VIRTUAL,
	          __std_vectorT1_lo_message___getState,
	          "",
	          "For each subclass of ReferencedNode, we override the getState() method to fill the vector with the correct set of methods for this particular node ");
	I_SimpleProperty(int, Active, 
	                 __int__getActive, 
	                 __void__setActive__int);
	I_SimpleProperty(const char *, Plugin, 
	                 __C5_char_P1__getPlugin, 
	                 __void__setPlugin__C5_char_P1);
	I_SimpleProperty(std::vector< lo_message >, State, 
	                 __std_vectorT1_lo_message___getState, 
	                 0);
	I_PublicMemberProperty(std::vector< SoundConnection * >, connectTO);
	I_PublicMemberProperty(std::vector< SoundConnection * >, connectFROM);
END_REFLECTOR
