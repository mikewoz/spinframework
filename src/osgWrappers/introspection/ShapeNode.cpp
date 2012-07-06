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
#include <ShapeNode.h>

// Must undefine IN and OUT macros defined in Windows headers
#ifdef IN
#undef IN
#endif
#ifdef OUT
#undef OUT
#endif

BEGIN_ENUM_REFLECTOR(spin::ShapeNode::shapeType)
	I_DeclaringFile("ShapeNode.h");
	I_EnumLabel(spin::ShapeNode::NONE);
	I_EnumLabel(spin::ShapeNode::SPHERE);
	I_EnumLabel(spin::ShapeNode::BOX);
	I_EnumLabel(spin::ShapeNode::CYLINDER);
	I_EnumLabel(spin::ShapeNode::CAPSULE);
	I_EnumLabel(spin::ShapeNode::CONE);
	I_EnumLabel(spin::ShapeNode::PLANE);
END_REFLECTOR

BEGIN_ENUM_REFLECTOR(spin::ShapeNode::billboardType)
	I_DeclaringFile("ShapeNode.h");
	I_EnumLabel(spin::ShapeNode::RELATIVE);
	I_EnumLabel(spin::ShapeNode::POINT_EYE);
	I_EnumLabel(spin::ShapeNode::STAY_UP);
END_REFLECTOR

BEGIN_OBJECT_REFLECTOR(spin::ShapeNode)
	I_DeclaringFile("ShapeNode.h");
	I_BaseType(spin::GroupNode);
	I_Constructor2(IN, spin::SceneManager *, sceneManager, IN, const char *, initID,
	               ____ShapeNode__SceneManager_P1__C5_char_P1,
	               "",
	               "");
	I_Method1(void, setContext, IN, const char *, newvalue,
	          Properties::VIRTUAL,
	          __void__setContext__C5_char_P1,
	          "",
	          "Returns the last stored nodepath (note: may have changed in current update traversal A node can 'belong' to a certain host machine, allowing it to be rendered or behave differently than on other machines.NOTE: the \"NULL\" string means that it belongs to no specific context.NOTE: a scene operating in SERVER_MODE will always create the node, so this feature is only really relevant for clients applications. ");
	I_Method1(void, setShape, IN, spin::ShapeNode::shapeType, t,
	          Properties::NON_VIRTUAL,
	          __void__setShape__shapeType,
	          "",
	          "Sets the shape this ShapeNode should have, identified by its number. ");
	I_Method1(void, setBillboard, IN, spin::ShapeNode::billboardType, t,
	          Properties::NON_VIRTUAL,
	          __void__setBillboard__billboardType,
	          "",
	          "");
	I_Method4(void, setColor, IN, float, red, IN, float, green, IN, float, blue, IN, float, alpha,
	          Properties::NON_VIRTUAL,
	          __void__setColor__float__float__float__float,
	          "",
	          " param red Red channel. Number in the range [0, 1]  green Green channel. Number in the range [0, 1]  blue Blue channel. Number in the range [0, 1]  alpha Opacity channel. Number in the range [0, 1]  ");
	I_Method1(void, setRenderBin, IN, int, i,
	          Properties::NON_VIRTUAL,
	          __void__setRenderBin__int,
	          "",
	          "");
	I_Method1(void, setLighting, IN, int, i,
	          Properties::NON_VIRTUAL,
	          __void__setLighting__int,
	          "",
	          "");
	I_Method1(void, setSingleSided, IN, int, singleSided,
	          Properties::NON_VIRTUAL,
	          __void__setSingleSided__int,
	          "",
	          "Specify whether both sides or only one side of the shape is rendered. ie, whether the backface is culled or not. ");
	I_Method0(int, getSingleSided,
	          Properties::NON_VIRTUAL,
	          __int__getSingleSided,
	          "",
	          "");
	I_Method0(void, updateStateSet,
	          Properties::VIRTUAL,
	          __void__updateStateSet,
	          "",
	          "This method actually applies the stateset to the subgraph, replacing any existing stateset with this one. The setStateSet and setStateSetFromFile methods just set the stateset_ symbol, while updateStateSet does the actual work.Override this method in subclasses in order to change how stateset should be applied. For example, to which node in the subgraph it should be attached, or whether it should be merged with the existing stateset (rather than merged).By default it is applied to the mainTransform_. ");
	I_Method0(int, getShape,
	          Properties::NON_VIRTUAL,
	          __int__getShape,
	          "",
	          "");
	I_Method0(int, getBillboard,
	          Properties::NON_VIRTUAL,
	          __int__getBillboard,
	          "",
	          "");
	I_Method0(osg::Vec4, getColor,
	          Properties::NON_VIRTUAL,
	          __osg_Vec4__getColor,
	          "",
	          "");
	I_Method0(int, getRenderBin,
	          Properties::NON_VIRTUAL,
	          __int__getRenderBin,
	          "",
	          "");
	I_Method0(int, getLighting,
	          Properties::NON_VIRTUAL,
	          __int__getLighting,
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
	I_SimpleProperty(int, Billboard, 
	                 __int__getBillboard, 
	                 0);
	I_SimpleProperty(osg::Vec4, Color, 
	                 __osg_Vec4__getColor, 
	                 0);
	I_SimpleProperty(const char *, Context, 
	                 0, 
	                 __void__setContext__C5_char_P1);
	I_SimpleProperty(int, Lighting, 
	                 __int__getLighting, 
	                 __void__setLighting__int);
	I_SimpleProperty(int, RenderBin, 
	                 __int__getRenderBin, 
	                 __void__setRenderBin__int);
	I_SimpleProperty(int, Shape, 
	                 __int__getShape, 
	                 0);
	I_SimpleProperty(int, SingleSided, 
	                 __int__getSingleSided, 
	                 __void__setSingleSided__int);
	I_SimpleProperty(std::vector< lo_message >, State, 
	                 __std_vectorT1_lo_message___getState, 
	                 0);
	I_PublicMemberProperty(spin::ShapeNode::shapeType, shape);
	I_PublicMemberProperty(spin::ShapeNode::billboardType, billboard);
	I_PublicMemberProperty(osg::Vec4, _color);
	I_PublicMemberProperty(std::string, texturePath);
	I_PublicMemberProperty(int, renderBin);
	I_PublicMemberProperty(bool, lightingEnabled);
	I_PublicMemberProperty(osg::ref_ptr< osg::Geode >, shapeGeode);
	I_PublicMemberProperty(osgUtil::Optimizer, optimizer);
END_REFLECTOR

