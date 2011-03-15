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

#include <ModelNode.h>
#include <SceneManager.h>

// Must undefine IN and OUT macros defined in Windows headers
#ifdef IN
#undef IN
#endif
#ifdef OUT
#undef OUT
#endif

BEGIN_ENUM_REFLECTOR(ModelNode::animationModeType)
	I_DeclaringFile("ModelNode.h");
	I_EnumLabel(ModelNode::OFF);
	I_EnumLabel(ModelNode::SWITCH);
	I_EnumLabel(ModelNode::SEQUENCE);
END_REFLECTOR

BEGIN_OBJECT_REFLECTOR(ModelNode)
	I_DeclaringFile("ModelNode.h");
	I_BaseType(GroupNode);
	I_Constructor2(IN, SceneManager *, sceneManager, IN, char *, initID,
	               ____ModelNode__SceneManager_P1__char_P1,
	               "",
	               "");
	I_Method1(void, setContext, IN, const char *, newvalue,
	          Properties::VIRTUAL,
	          __void__setContext__C5_char_P1,
	          "",
	          "The context is an arbitrary keyword that associates this node with a particular behaviour. Currently, it is used to *prevent* display if the context matches the name of a machine. ie, allowing it to be seen on all machines except for the one that is named by setContext. ");
	I_Method1(void, setModelFromFile, IN, const char *, filename,
	          Properties::NON_VIRTUAL,
	          __void__setModelFromFile__C5_char_P1,
	          "",
	          "Load a 3D model from a file (eg, .osg, .3ds, .obj, .dae, etc). Make sure that StateRegistration flag is set if you want to have control over any textures or shaders withing the model ");
	I_Method0(const char *, getModelFromFile,
	          Properties::NON_VIRTUAL,
	          __C5_char_P1__getModelFromFile,
	          "",
	          "");
	I_Method1(void, setStateRegistration, IN, int, i,
	          Properties::NON_VIRTUAL,
	          __void__setStateRegistration__int,
	          "",
	          "The StateRegistration flag should be set if you want any textures or shaders to be parsed out when loading a model. Any statesets found in the file will generate corresponding ReferencedStateSets for use within SPIN. This way, you'll be able to swap textures, control videos, adjust shader parameters, etc. ");
	I_Method0(int, getStateRegistration,
	          Properties::NON_VIRTUAL,
	          __int__getStateRegistration,
	          "",
	          "");
	I_Method1(void, setRenderBin, IN, int, i,
	          Properties::NON_VIRTUAL,
	          __void__setRenderBin__int,
	          "",
	          "Render bins allow you to control drawing order, and manage Z-fighting. The higher the number, the later it gets processed (ie, appears on top). Default renderBin = 11 ");
	I_Method0(int, getRenderBin,
	          Properties::NON_VIRTUAL,
	          __int__getRenderBin,
	          "",
	          "");
	I_Method2(void, setKeyframe, IN, int, index, IN, float, keyframe,
	          Properties::NON_VIRTUAL,
	          __void__setKeyframe__int__float,
	          "",
	          "Render bins allow you to control drawing order, and manage Z-fighting. The higher the number, the later it gets processed (ie, appears on top). Default renderBin = 11 ");
	I_Method1(float, getKeyframe, IN, int, index,
	          Properties::NON_VIRTUAL,
	          __float__getKeyframe__int,
	          "",
	          "");
	I_Method2(void, setStateSet, IN, int, index, IN, const char *, replacement,
	          Properties::NON_VIRTUAL,
	          __void__setStateSet__int__C5_char_P1,
	          "",
	          "For statesets embedded in the model, it is possible to swap with some other (already existing) stateset.Note: for this to work, stateRegistration must be enabled. ");
	I_Method0(std::vector< lo_message >, getState,
	          Properties::VIRTUAL,
	          __std_vectorT1_lo_message___getState,
	          "",
	          "For each subclass of ReferencedNode, we override the getState() method to fill the vector with the correct set of methods for this particular node ");
	I_SimpleProperty(const char *, Context, 
	                 0, 
	                 __void__setContext__C5_char_P1);
	I_IndexedProperty(float, Keyframe, 
	                  __float__getKeyframe__int, 
	                  __void__setKeyframe__int__float, 
	                  0);
	I_SimpleProperty(const char *, ModelFromFile, 
	                 __C5_char_P1__getModelFromFile, 
	                 __void__setModelFromFile__C5_char_P1);
	I_SimpleProperty(int, RenderBin, 
	                 __int__getRenderBin, 
	                 __void__setRenderBin__int);
	I_SimpleProperty(std::vector< lo_message >, State, 
	                 __std_vectorT1_lo_message___getState, 
	                 0);
	I_SimpleProperty(int, StateRegistration, 
	                 __int__getStateRegistration, 
	                 __void__setStateRegistration__int);
	I_PublicMemberProperty(std::vector< t_symbol * >, _statesetList);
END_REFLECTOR

