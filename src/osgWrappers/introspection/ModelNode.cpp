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

BEGIN_ENUM_REFLECTOR(spin::ModelNode::animationModeType)
	I_DeclaringFile("ModelNode.h");
	I_EnumLabel(spin::ModelNode::OFF);
	I_EnumLabel(spin::ModelNode::SWITCH);
	I_EnumLabel(spin::ModelNode::SEQUENCE);
END_REFLECTOR

BEGIN_OBJECT_REFLECTOR(spin::ModelNode)
	I_DeclaringFile("ModelNode.h");
	I_BaseType(spin::GroupNode);
	I_Constructor2(IN, spin::SceneManager *, sceneManager, IN, char *, initID,
	               ____ModelNode__SceneManager_P1__char_P1,
	               "",
	               "");
	I_MethodWithDefaults1(void, updateNodePath, IN, bool, updateChildren, true,
	                      Properties::VIRTUAL,
	                      __void__updateNodePath__bool,
	                      "",
	                      "We change our attachmentNode (add attachment to the centroid), so we MUST override updateNodePath(), and manually push the centroid transform onto the currentNodePath. ");
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
	I_Method1(void, setAttachCentroid, IN, int, i,
	          Properties::NON_VIRTUAL,
	          __void__setAttachCentroid__int,
	          "",
	          "If attachCentroid is enabled, then children will be attached to the centroid of the currently loaded model. If not then it will be attached to this ModelNode's local origin. ");
	I_Method0(int, getAttachCentroid,
	          Properties::NON_VIRTUAL,
	          __int__getAttachCentroid,
	          "",
	          "");
	I_Method0(void, makeCentered,
	          Properties::NON_VIRTUAL,
	          __void__makeCentered,
	          "",
	          "Translate the model so that the centroid is at the local (0,0,0) ");
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
	          "Control the keyframe of a particular animation saved within the model (there can be several animations, hence the required index number) ");
	I_Method1(float, getKeyframe, IN, int, index,
	          Properties::NON_VIRTUAL,
	          __float__getKeyframe__int,
	          "",
	          "");
	I_Method2(void, setPlaying, IN, int, index, IN, int, playState,
	          Properties::NON_VIRTUAL,
	          __void__setPlaying__int__int,
	          "",
	          "Set the playing state of a particular animation (paused by default) ");
	I_Method1(float, getPlaying, IN, int, index,
	          Properties::NON_VIRTUAL,
	          __float__getPlaying__int,
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
	I_Method1(void, setLighting, IN, int, i,
	          Properties::NON_VIRTUAL,
	          __void__setLighting__int,
	          "",
	          "This lets you enable or disable the lighting for the entire model, BUT, really this should be done in individual statesets and can be overridden ");
	I_Method0(int, getLighting,
	          Properties::NON_VIRTUAL,
	          __int__getLighting,
	          "",
	          "");
	I_SimpleProperty(int, AttachCentroid, 
	                 __int__getAttachCentroid, 
	                 __void__setAttachCentroid__int);
	I_SimpleProperty(const char *, Context, 
	                 0, 
	                 __void__setContext__C5_char_P1);
	I_IndexedProperty(float, Keyframe, 
	                  __float__getKeyframe__int, 
	                  __void__setKeyframe__int__float, 
	                  0);
	I_SimpleProperty(int, Lighting, 
	                 __int__getLighting, 
	                 __void__setLighting__int);
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
	I_PublicMemberProperty(std::vector< spin::t_symbol * >, _statesetList);
END_REFLECTOR

