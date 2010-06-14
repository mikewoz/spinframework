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

#include <ConstraintsNode.h>
#include <SceneManager.h>

// Must undefine IN and OUT macros defined in Windows headers
#ifdef IN
#undef IN
#endif
#ifdef OUT
#undef OUT
#endif

BEGIN_ENUM_REFLECTOR(ConstraintsNode::constraintMode)
	I_DeclaringFile("ConstraintsNode.h");
	I_EnumLabel(ConstraintsNode::BASIC);
	I_EnumLabel(ConstraintsNode::DROP);
	I_EnumLabel(ConstraintsNode::COLLIDE);
	I_EnumLabel(ConstraintsNode::BOUNCE);
END_REFLECTOR

BEGIN_OBJECT_REFLECTOR(ConstraintsNode)
	I_DeclaringFile("ConstraintsNode.h");
	I_BaseType(GroupNode);
	I_Constructor2(IN, SceneManager *, sceneManager, IN, char *, initID,
	               ____ConstraintsNode__SceneManager_P1__char_P1,
	               "",
	               "");
	I_Method0(void, callbackUpdate,
	          Properties::VIRTUAL,
	          __void__callbackUpdate,
	          "",
	          "For nodes that require regular programmatic control, there is a callback that is evaluated with every refresh. This function can thus be used for animations, or any other periodic updates.Note that changes to the scene graph structure (eg, moving/deleting nodes should NOT be done within this callback because traversals stacks will become corrupted. The technique is rather to enable a flag and then do the actual change in the SceneManager::updateGraph() method. ");
	I_Method1(void, setTarget, IN, const char *, id,
	          Properties::NON_VIRTUAL,
	          __void__setTarget__C5_char_P1,
	          "",
	          "");
	I_Method0(const char *, getTarget,
	          Properties::NON_VIRTUAL,
	          __C5_char_P1__getTarget,
	          "",
	          "");
	I_Method1(void, setConstraintMode, IN, ConstraintsNode::constraintMode, m,
	          Properties::NON_VIRTUAL,
	          __void__setConstraintMode__C5raintMode,
	          "",
	          "");
	I_Method0(int, getConstraintMode,
	          Properties::NON_VIRTUAL,
	          __int__getConstraintMode,
	          "",
	          "");
	I_Method3(void, setCubeSize, IN, float, xScale, IN, float, yScale, IN, float, zScale,
	          Properties::NON_VIRTUAL,
	          __void__setCubeSize__float__float__float,
	          "",
	          "");
	I_Method3(void, setCubeOffset, IN, float, x, IN, float, y, IN, float, z,
	          Properties::NON_VIRTUAL,
	          __void__setCubeOffset__float__float__float,
	          "",
	          "");
	I_Method0(osg::Vec3, getCubeSize,
	          Properties::NON_VIRTUAL,
	          __osg_Vec3__getCubeSize,
	          "",
	          "");
	I_Method0(osg::Vec3, getCubeOffset,
	          Properties::NON_VIRTUAL,
	          __osg_Vec3__getCubeOffset,
	          "",
	          "");
	I_Method3(void, setTranslation, IN, float, x, IN, float, y, IN, float, z,
	          Properties::VIRTUAL,
	          __void__setTranslation__float__float__float,
	          "",
	          "The local translation offset for this node with respect to it's parent ");
	I_Method3(void, translate, IN, float, x, IN, float, y, IN, float, z,
	          Properties::VIRTUAL,
	          __void__translate__float__float__float,
	          "",
	          "The translate command increments the node's current translation values (ie, it's position in the scene with respect to it's parent) ");
	I_Method3(void, move, IN, float, x, IN, float, y, IN, float, z,
	          Properties::VIRTUAL,
	          __void__move__float__float__float,
	          "",
	          "The move command adds a relative translation with respect to the node's current orientation. That is, the node will translate along it's direction vector by the supplied number of units. ");
	I_Method1(void, applyConstrainedTranslation, IN, osg::Vec3, v,
	          Properties::NON_VIRTUAL,
	          __void__applyConstrainedTranslation__osg_Vec3,
	          "",
	          "A pseudo-recursive function that checks if a translation results in one or more intersections with the target node. If no intersection, this method defaults to just a setTranslation call. Otherwise, it will do a setTranslation for the collision point, and call itself again until there are no collisions left. ");
	I_Method0(std::vector< lo_message >, getState,
	          Properties::VIRTUAL,
	          __std_vectorT1_lo_message___getState,
	          "",
	          "For each subclass of ReferencedNode, we override the getState() method to fill the vector with the correct set of methods for this particular node ");
	I_SimpleProperty(int, ConstraintMode, 
	                 __int__getConstraintMode, 
	                 0);
	I_SimpleProperty(osg::Vec3, CubeOffset, 
	                 __osg_Vec3__getCubeOffset, 
	                 0);
	I_SimpleProperty(osg::Vec3, CubeSize, 
	                 __osg_Vec3__getCubeSize, 
	                 0);
	I_SimpleProperty(std::vector< lo_message >, State, 
	                 __std_vectorT1_lo_message___getState, 
	                 0);
	I_SimpleProperty(const char *, Target, 
	                 __C5_char_P1__getTarget, 
	                 __void__setTarget__C5_char_P1);
END_REFLECTOR

