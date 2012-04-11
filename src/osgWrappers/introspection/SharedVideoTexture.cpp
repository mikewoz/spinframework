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

#include <SceneManager.h>
#include <SharedVideoTexture.h>

// Must undefine IN and OUT macros defined in Windows headers
#ifdef IN
#undef IN
#endif
#ifdef OUT
#undef OUT
#endif

BEGIN_OBJECT_REFLECTOR(spin::SharedVideoTexture)
	I_DeclaringFile("SharedVideoTexture.h");
	I_BaseType(spin::ReferencedStateSet);
	I_Constructor2(IN, spin::SceneManager *, sceneManager, IN, const char *, initID,
	               ____SharedVideoTexture__SceneManager_P1__C5_char_P1,
	               "",
	               "");
	I_Method1(void, setTextureID, IN, const char *, id,
	          Properties::NON_VIRTUAL,
	          __void__setTextureID__C5_char_P1,
	          "",
	          "");
	I_Method0(const char *, getTextureID,
	          Properties::NON_VIRTUAL,
	          __C5_char_P1__getTextureID,
	          "",
	          "");
	I_Method0(const char *, getPath,
	          Properties::VIRTUAL,
	          __C5_char_P1__getPath,
	          "",
	          "Abstract method getPath needs to be implemented ");
	I_Method0(std::vector< lo_message >, getState,
	          Properties::VIRTUAL,
	          __std_vectorT1_lo_message___getState,
	          "",
	          "Just like a ReferencedNode, each subclass of ReferencedStateSet must override the getState() method to pass it's current state. ");
	I_Method0(void, debug,
	          Properties::VIRTUAL,
	          __void__debug,
	          "",
	          "Print debug information to console. ");
	I_Method0(void, updateCallback,
	          Properties::VIRTUAL,
	          __void__updateCallback,
	          "",
	          "This callback occurs every frame to update the state with any parameter changes. ");
	I_Method0(void, consumeFrame,
	          Properties::NON_VIRTUAL,
	          __void__consumeFrame,
	          "",
	          "");
	I_Method0(void, signalKilled,
	          Properties::NON_VIRTUAL,
	          __void__signalKilled,
	          "",
	          "");
	I_Method0(void, start,
	          Properties::NON_VIRTUAL,
	          __void__start,
	          "",
	          "");
	I_Method0(void, stop,
	          Properties::NON_VIRTUAL,
	          __void__stop,
	          "",
	          "");
	I_SimpleProperty(const char *, Path, 
	                 __C5_char_P1__getPath, 
	                 0);
	I_SimpleProperty(std::vector< lo_message >, State, 
	                 __std_vectorT1_lo_message___getState, 
	                 0);
	I_SimpleProperty(const char *, TextureID, 
	                 __C5_char_P1__getTextureID, 
	                 __void__setTextureID__C5_char_P1);
END_REFLECTOR

