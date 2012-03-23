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
#include <TextNode.h>

// Must undefine IN and OUT macros defined in Windows headers
#ifdef IN
#undef IN
#endif
#ifdef OUT
#undef OUT
#endif

BEGIN_ENUM_REFLECTOR(spin::TextNode::billboardType)
	I_DeclaringFile("TextNode.h");
	I_EnumLabel(spin::TextNode::RELATIVE);
	I_EnumLabel(spin::TextNode::POINT_EYE);
	I_EnumLabel(spin::TextNode::STAY_UP);
END_REFLECTOR

BEGIN_ENUM_REFLECTOR(spin::TextNode::decorationType)
	I_DeclaringFile("TextNode.h");
	I_EnumLabel(spin::TextNode::DROP_SHADOW_BOTTOM_RIGHT);
	I_EnumLabel(spin::TextNode::DROP_SHADOW_CENTER_RIGHT);
	I_EnumLabel(spin::TextNode::DROP_SHADOW_TOP_RIGHT);
	I_EnumLabel(spin::TextNode::DROP_SHADOW_BOTTOM_CENTER);
	I_EnumLabel(spin::TextNode::DROP_SHADOW_TOP_CENTER);
	I_EnumLabel(spin::TextNode::DROP_SHADOW_BOTTOM_LEFT);
	I_EnumLabel(spin::TextNode::DROP_SHADOW_CENTER_LEFT);
	I_EnumLabel(spin::TextNode::DROP_SHADOW_TOP_LEFT);
	I_EnumLabel(spin::TextNode::OUTLINE);
	I_EnumLabel(spin::TextNode::NONE);
END_REFLECTOR

BEGIN_ENUM_REFLECTOR(spin::TextNode::backgroundType)
	I_DeclaringFile("TextNode.h");
	I_EnumLabel(spin::TextNode::NO_BACKGROUND);
	I_EnumLabel(spin::TextNode::FILLED);
	I_EnumLabel(spin::TextNode::WIREFRAME);
	I_EnumLabel(spin::TextNode::ALL);
END_REFLECTOR

BEGIN_OBJECT_REFLECTOR(spin::TextNode)
	I_DeclaringFile("TextNode.h");
	I_BaseType(spin::GroupNode);
	I_Constructor2(IN, spin::SceneManager *, sceneManager, IN, char *, initID,
	               ____TextNode__SceneManager_P1__char_P1,
	               "",
	               "");
	I_Method1(void, setContext, IN, const char *, newvalue,
	          Properties::VIRTUAL,
	          __void__setContext__C5_char_P1,
	          "",
	          "A node can 'belong' to a certain host machine, allowing it to be rendered or behave differently than on other machines.NOTE: the \"NULL\" string means that it belongs to no specific context.NOTE: a scene operating in SERVER_MODE will always create the node, so this feature is only really relevant for clients applications. ");
	I_Method1(void, setTextValue, IN, const char *, s,
	          Properties::NON_VIRTUAL,
	          __void__setTextValue__C5_char_P1,
	          "",
	          "Accepts user-entered string for the node's text. ");
	I_Method1(void, setFont, IN, const char *, s,
	          Properties::NON_VIRTUAL,
	          __void__setFont__C5_char_P1,
	          "",
	          "Sets the font for the text associated with this node. ");
	I_Method1(void, setSize, IN, float, s,
	          Properties::NON_VIRTUAL,
	          __void__setSize__float,
	          "",
	          "Sets the point-size for the text associated with this node. ");
	I_Method4(void, setColor, IN, float, red, IN, float, green, IN, float, blue, IN, float, alpha,
	          Properties::NON_VIRTUAL,
	          __void__setColor__float__float__float__float,
	          "",
	          "Sets the color for the text associated to this node in RGBA values. ");
	I_Method4(void, setBgColor, IN, float, red, IN, float, green, IN, float, blue, IN, float, alpha,
	          Properties::NON_VIRTUAL,
	          __void__setBgColor__float__float__float__float,
	          "",
	          "Sets the background color for this node. ");
	I_Method1(void, setMargin, IN, float, margin,
	          Properties::NON_VIRTUAL,
	          __void__setMargin__float,
	          "",
	          "Sets the margins for the text associated to this node. ");
	I_Method1(void, setBillboard, IN, spin::TextNode::billboardType, t,
	          Properties::NON_VIRTUAL,
	          __void__setBillboard__billboardType,
	          "",
	          "Sets the type of billboarding asigned to this node (drawn from the enum billboardType). ");
	I_Method1(void, setDecoration, IN, spin::TextNode::decorationType, t,
	          Properties::NON_VIRTUAL,
	          __void__setDecoration__decorationType,
	          "",
	          "Sets the shadowing or outline type for this text node (drawn from the decorationType enum). ");
	I_Method1(void, setBackground, IN, spin::TextNode::backgroundType, t,
	          Properties::NON_VIRTUAL,
	          __void__setBackground__backgroundType,
	          "",
	          "Sets a background type for the text box (drawn from the backgroundType enum). ");
	I_Method0(const char *, getTextValue,
	          Properties::NON_VIRTUAL,
	          __C5_char_P1__getTextValue,
	          "",
	          "Returns a string with the text associated to this node. ");
	I_Method0(std::string, getTextString,
	          Properties::NON_VIRTUAL,
	          __std_string__getTextString,
	          "",
	          "Returns a string with the text associated to this node. ");
	I_Method0(const char *, getFont,
	          Properties::NON_VIRTUAL,
	          __C5_char_P1__getFont,
	          "",
	          "Returns a string indicating the font of the text associated to this node. ");
	I_Method0(float, getSize,
	          Properties::NON_VIRTUAL,
	          __float__getSize,
	          "",
	          "Returns a float indicating the size of the text associated to this node. ");
	I_Method0(osg::Vec4, getColor,
	          Properties::NON_VIRTUAL,
	          __osg_Vec4__getColor,
	          "",
	          "Returns the color (in RGBA values) of the text associated to this node. ");
	I_Method0(osg::Vec4, getBgColor,
	          Properties::NON_VIRTUAL,
	          __osg_Vec4__getBgColor,
	          "",
	          "Returns the background color of the text box in RGBA values. ");
	I_Method0(float, getMargin,
	          Properties::NON_VIRTUAL,
	          __float__getMargin,
	          "",
	          "Returns a float indicating the margin size of the text box. ");
	I_Method0(int, getBillboard,
	          Properties::NON_VIRTUAL,
	          __int__getBillboard,
	          "",
	          "Returns the currently set billboarding type with respect to the billboardType enum. ");
	I_Method0(int, getDecoration,
	          Properties::NON_VIRTUAL,
	          __int__getDecoration,
	          "",
	          "Returns the currently set decoration type (shadows or outlines) with respect to the decorationType enum. ");
	I_Method0(int, getBackround,
	          Properties::NON_VIRTUAL,
	          __int__getBackround,
	          "",
	          "Returns the currently set background type with respect to the choices in the backgroundType enum. ");
	I_Method0(std::vector< lo_message >, getState,
	          Properties::VIRTUAL,
	          __std_vectorT1_lo_message___getState,
	          "",
	          "For each subclass of ReferencedNode, we override the getState() method to fill the vector with the correct set of methods for this particular node ");
	I_SimpleProperty(spin::TextNode::backgroundType, Background, 
	                 0, 
	                 __void__setBackground__backgroundType);
	I_SimpleProperty(int, Backround, 
	                 __int__getBackround, 
	                 0);
	I_SimpleProperty(osg::Vec4, BgColor, 
	                 __osg_Vec4__getBgColor, 
	                 0);
	I_SimpleProperty(int, Billboard, 
	                 __int__getBillboard, 
	                 0);
	I_SimpleProperty(osg::Vec4, Color, 
	                 __osg_Vec4__getColor, 
	                 0);
	I_SimpleProperty(const char *, Context, 
	                 0, 
	                 __void__setContext__C5_char_P1);
	I_SimpleProperty(int, Decoration, 
	                 __int__getDecoration, 
	                 0);
	I_SimpleProperty(const char *, Font, 
	                 __C5_char_P1__getFont, 
	                 __void__setFont__C5_char_P1);
	I_SimpleProperty(float, Margin, 
	                 __float__getMargin, 
	                 __void__setMargin__float);
	I_SimpleProperty(float, Size, 
	                 __float__getSize, 
	                 __void__setSize__float);
	I_SimpleProperty(std::vector< lo_message >, State, 
	                 __std_vectorT1_lo_message___getState, 
	                 0);
	I_SimpleProperty(std::string, TextString, 
	                 __std_string__getTextString, 
	                 0);
	I_SimpleProperty(const char *, TextValue, 
	                 __C5_char_P1__getTextValue, 
	                 __void__setTextValue__C5_char_P1);
END_REFLECTOR

