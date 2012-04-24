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

#include <CollisionShape.h>
#include <SceneManager.h>

// Must undefine IN and OUT macros defined in Windows headers
#ifdef IN
#undef IN
#endif
#ifdef OUT
#undef OUT
#endif

BEGIN_OBJECT_REFLECTOR(spin::CollisionShape)
	I_DeclaringFile("CollisionShape.h");
	I_BaseType(spin::ShapeNode);
	I_Constructor2(IN, spin::SceneManager *, sceneManager, IN, char *, initID,
	               ____CollisionShape__SceneManager_P1__char_P1,
	               "",
	               "");
	I_Method0(void, callbackUpdate,
	          Properties::VIRTUAL,
	          __void__callbackUpdate,
	          "",
	          "For nodes that require regular programmatic control, there is a callback that is evaluated with every refresh. This function can thus be used for animations, or any other periodic updates.Note that changes to the scene graph structure (eg, moving/deleting nodes should NOT be done within this callback because traversals stacks will become corrupted. The technique is rather to enable a flag and then do the actual change in the SceneManager::updateGraph() method. ");
	I_Method0(void, debug,
	          Properties::VIRTUAL,
	          __void__debug,
	          "",
	          "Debug print (to log/console) ");
	I_Method1(void, setMass, IN, float, mass,
	          Properties::NON_VIRTUAL,
	          __void__setMass__float,
	          "",
	          "Set the mass of the object. A value of 0 makes the object static (unmovable) by the dynamics engine. Only direct transformation through GroupNode functions will have any effect. ");
	I_Method0(float, getMass,
	          Properties::NON_VIRTUAL,
	          __float__getMass,
	          "",
	          "");
	I_Method1(void, setDynamic, IN, int, isDynamic,
	          Properties::NON_VIRTUAL,
	          __void__setDynamic__int,
	          "",
	          "Enable external dynamic forces (eg, gravity). Note: object must have non-zero mass for this to work. ");
	I_Method0(int, getDynamic,
	          Properties::NON_VIRTUAL,
	          __int__getDynamic,
	          "",
	          "");
	I_Method3(void, setOrientation, IN, float, pitch, IN, float, roll, IN, float, yaw,
	          Properties::VIRTUAL,
	          __void__setOrientation__float__float__float,
	          "",
	          "The local orientation offset for this node with respect to it's parent ");
	I_Method3(void, setTranslation, IN, float, x, IN, float, y, IN, float, z,
	          Properties::VIRTUAL,
	          __void__setTranslation__float__float__float,
	          "",
	          "The local translation offset for this node with respect to it's parent ");
	I_Method3(void, setScale, IN, float, x, IN, float, y, IN, float, z,
	          Properties::VIRTUAL,
	          __void__setScale__float__float__float,
	          "",
	          "A grouped scale operation ");
	I_Method16(void, setManipulatorMatrix, IN, float, a00, IN, float, a01, IN, float, a02, IN, float, a03, IN, float, a10, IN, float, a11, IN, float, a12, IN, float, a13, IN, float, a20, IN, float, a21, IN, float, a22, IN, float, a23, IN, float, a30, IN, float, a31, IN, float, a32, IN, float, a33,
	           Properties::VIRTUAL,
	           __void__setManipulatorMatrix__float__float__float__float__float__float__float__float__float__float__float__float__float__float__float__float,
	           "",
	           "");
	I_Method0(std::vector< lo_message >, getState,
	          Properties::VIRTUAL,
	          __std_vectorT1_lo_message___getState,
	          "",
	          "For each subclass of ReferencedNode, we override the getState() method to fill the vector with the correct set of methods for this particular node ");
	I_ProtectedMethod0(void, drawShape,
	                   Properties::VIRTUAL,
	                   Properties::NON_CONST,
	                   __void__drawShape,
	                   "",
	                   "");
	I_SimpleProperty(int, Dynamic, 
	                 __int__getDynamic, 
	                 __void__setDynamic__int);
	I_SimpleProperty(float, Mass, 
	                 __float__getMass, 
	                 __void__setMass__float);
	I_SimpleProperty(std::vector< lo_message >, State, 
	                 __std_vectorT1_lo_message___getState, 
	                 0);
END_REFLECTOR

