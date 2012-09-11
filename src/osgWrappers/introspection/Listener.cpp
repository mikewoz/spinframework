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

#include <Listener.h>
#include <SceneManager.h>

// Must undefine IN and OUT macros defined in Windows headers
#ifdef IN
#undef IN
#endif
#ifdef OUT
#undef OUT
#endif

BEGIN_OBJECT_REFLECTOR(spin::Listener)
	I_DeclaringFile("Listener.h");
	I_BaseType(spin::DSPNode);
	I_Constructor2(IN, spin::SceneManager *, sceneManager, IN, const char *, initID,
	               ____Listener__SceneManager_P1__C5_char_P1,
	               "",
	               "");
	I_Method0(void, debug,
	          Properties::VIRTUAL,
	          __void__debug,
	          "",
	          "Print debug information about the node to standard out (when running in console mode). It may be possible to redirect this to a text box for GUI logs. ");
	I_Method1(void, callbackUpdate, IN, osg::NodeVisitor *, nv,
	          Properties::VIRTUAL,
	          __void__callbackUpdate__osg_NodeVisitor_P1,
	          "",
	          "For nodes that require regular programmatic control, there is a callback that is evaluated with every refresh. This function can thus be used for animations, or any other periodic updates.Note that changes to the scene graph structure (eg, moving/deleting nodes should NOT be done within this callback because traversals stacks will become corrupted. The technique is rather to enable a flag and then do the actual change in the SceneManager::updateGraph() method. ");
	I_MethodWithDefaults1(bool, dumpGlobals, IN, bool, forced, false,
	                      Properties::VIRTUAL,
	                      __bool__dumpGlobals__bool,
	                      "",
	                      "The dumpGlobals method results in a broadcast of this node's translation and orientation. It is called by callbackUpdate() every frame, however the 'forced' flag will be set to false, so it will only send a message if the node's matrix has changed. If the 'forced' flag is set to true, it will definitely result in a message broadcast. This should only be used when necessary (eg, when a stateDump is requested).Note: the return value is only to fool wx so that it doesn't consider this as an editable property. ");
	I_Method0(std::vector< lo_message >, getState,
	          Properties::VIRTUAL,
	          __std_vectorT1_lo_message___getState,
	          "",
	          "For each subclass of ReferencedNode, we override the getState() method to fill the vector with the correct set of methods for this particular node ");
	I_Method2(void, setParam, IN, const char *, paramName, IN, const char *, paramValue,
	          Properties::VIRTUAL,
	          __void__setParam__C5_char_P1__C5_char_P1,
	          "",
	          "");
	I_Method2(void, setParam, IN, const char *, paramName, IN, float, paramValue,
	          Properties::VIRTUAL,
	          __void__setParam__C5_char_P1__float,
	          "",
	          "");
	I_Method3(void, setTranslation, IN, float, x, IN, float, y, IN, float, z,
	          Properties::VIRTUAL,
	          __void__setTranslation__float__float__float,
	          "",
	          "The local translation offset for this node with respect to it's parent ");
	I_Method3(void, setOrientation, IN, float, pitch, IN, float, roll, IN, float, yaw,
	          Properties::VIRTUAL,
	          __void__setOrientation__float__float__float,
	          "",
	          "The local orientation offset for this node with respect to it's parent ");
	I_Method4(void, setOrientationQuat, IN, float, x, IN, float, y, IN, float, z, IN, float, w,
	          Properties::VIRTUAL,
	          __void__setOrientationQuat__float__float__float__float,
	          "",
	          "Set the orientation offset as a quaternion ");
	I_Method1(void, setURI, IN, const char *, uri,
	          Properties::VIRTUAL,
	          __void__setURI__C5_char_P1,
	          "",
	          "Set the media for the sound node using a URI pattern.Examples: file://soundfilename.wav file:///home/johndoe/soundfilename.wav http://www.server.com/soundfile.wav adc://1:1 adc://1 content://media/external/audio/media/710 mms://some_media_stream rtsp://127.0.0.1:12311 pd_plugin://audio_plugin_patch.pd ");
	I_SimpleProperty(std::vector< lo_message >, State, 
	                 __std_vectorT1_lo_message___getState, 
	                 0);
	I_SimpleProperty(const char *, URI, 
	                 0, 
	                 __void__setURI__C5_char_P1);
END_REFLECTOR

