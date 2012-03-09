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
#include <VideoTexture.h>

// Must undefine IN and OUT macros defined in Windows headers
#ifdef IN
#undef IN
#endif
#ifdef OUT
#undef OUT
#endif

BEGIN_OBJECT_REFLECTOR(spin::VideoTexture)
	I_DeclaringFile("VideoTexture.h");
	I_BaseType(spin::ReferencedStateSet);
	I_Constructor2(IN, spin::SceneManager *, sceneManager, IN, const char *, initID,
	               ____VideoTexture__SceneManager_P1__C5_char_P1,
	               "",
	               "");
	I_Method0(void, debug,
	          Properties::VIRTUAL,
	          __void__debug,
	          "",
	          "Print debug information to console. ");
	I_Method1(void, setPath, IN, const char *, newPath,
	          Properties::NON_VIRTUAL,
	          __void__setPath__C5_char_P1,
	          "",
	          "Creates a video from a path on disk. This can either be a single movie file (ie, a format that OSG knows how to read), or a folder name that contains a sequence of images to stitch into a video. ");
	I_Method0(const char *, getPath,
	          Properties::VIRTUAL,
	          __C5_char_P1__getPath,
	          "",
	          "Returns a string indicating the path from which the video data is drawn. This is an abstract class and should be re-implemented in a derived class ");
	I_Method1(void, setLoop, IN, int, i,
	          Properties::NON_VIRTUAL,
	          __void__setLoop__int,
	          "",
	          "Enable (1) or disable (0) looping of the video ");
	I_Method0(int, getLoop,
	          Properties::NON_VIRTUAL,
	          __int__getLoop,
	          "",
	          "Returns a boolean indicating whether the video is set to loop or not. ");
	I_Method1(void, setIndex, IN, float, f,
	          Properties::NON_VIRTUAL,
	          __void__setIndex__float,
	          "",
	          "Normalized seek to a part of the video. ie, index in range [0,1] ");
	I_Method0(float, getIndex,
	          Properties::NON_VIRTUAL,
	          __float__getIndex,
	          "",
	          "Returns an index in the range 0,1 indicating a part of the video set to be rapidly accessed with the setIndex function. ");
	I_Method1(void, setFrameRate, IN, float, f,
	          Properties::NON_VIRTUAL,
	          __void__setFrameRate__float,
	          "",
	          "Only for image sequences; tells OSG how fast to play the sequence ");
	I_Method0(float, getFrameRate,
	          Properties::NON_VIRTUAL,
	          __float__getFrameRate,
	          "",
	          "Returns an integer indicating the set framerate of the video sequence. ");
	I_Method1(void, setPlay, IN, int, i,
	          Properties::NON_VIRTUAL,
	          __void__setPlay__int,
	          "",
	          "Play (1) or Pause (0) the video ");
	I_Method0(int, getPlay,
	          Properties::NON_VIRTUAL,
	          __int__getPlay,
	          "",
	          "Returns a boolean indicating whether the video is set to play or not. ");
	I_Method0(void, rewind,
	          Properties::NON_VIRTUAL,
	          __void__rewind,
	          "",
	          "Seek to beginning of video (equivalent to setIndex(0); ");
	I_Method0(void, flipHorizontal,
	          Properties::NON_VIRTUAL,
	          __void__flipHorizontal,
	          "",
	          "Flips the video horizontally. ");
	I_Method0(void, flipVertical,
	          Properties::NON_VIRTUAL,
	          __void__flipVertical,
	          "",
	          "Flips the video vertically. ");
	I_Method0(bool, isValid,
	          Properties::NON_VIRTUAL,
	          __bool__isValid,
	          "",
	          "Returns whether there is a currently valid video ");
	I_Method0(std::vector< lo_message >, getState,
	          Properties::VIRTUAL,
	          __std_vectorT1_lo_message___getState,
	          "",
	          "Just like a ReferencedNode, each subclass of ReferencedStateSet must override the getState() method to pass it's current state. ");
	I_SimpleProperty(float, FrameRate, 
	                 __float__getFrameRate, 
	                 __void__setFrameRate__float);
	I_SimpleProperty(float, Index, 
	                 __float__getIndex, 
	                 __void__setIndex__float);
	I_SimpleProperty(int, Loop, 
	                 __int__getLoop, 
	                 __void__setLoop__int);
	I_SimpleProperty(const char *, Path, 
	                 __C5_char_P1__getPath, 
	                 __void__setPath__C5_char_P1);
	I_SimpleProperty(int, Play, 
	                 __int__getPlay, 
	                 __void__setPlay__int);
	I_SimpleProperty(std::vector< lo_message >, State, 
	                 __std_vectorT1_lo_message___getState, 
	                 0);
END_REFLECTOR

