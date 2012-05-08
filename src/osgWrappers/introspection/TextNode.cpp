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

BEGIN_VALUE_REFLECTOR(spin::spinTextNode)
	I_DeclaringFile("TextNode.h");
	I_Constructor0(____spinTextNode,
	               "",
	               "");
	I_ProtectedMethod3(osgText::String::iterator, computeLastCharacterOnLine, IN, osg::Vec2 &, cursor, IN, osgText::String::iterator, first, IN, osgText::String::iterator, last,
	                   Properties::NON_VIRTUAL,
	                   Properties::NON_CONST,
	                   __osgText_String_iterator__computeLastCharacterOnLine__osg_Vec2_R1__osgText_String_iterator__osgText_String_iterator,
	                   "",
	                   "");
	I_ProtectedMethod0(void, computeGlyphRepresentation,
	                   Properties::NON_VIRTUAL,
	                   Properties::NON_CONST,
	                   __void__computeGlyphRepresentation,
	                   "",
	                   "");
END_REFLECTOR

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
	I_Method0(void, callbackUpdate,
	          Properties::VIRTUAL,
	          __void__callbackUpdate,
	          "",
	          "For nodes that require regular programmatic control, there is a callback that is evaluated with every refresh. This function can thus be used for animations, or any other periodic updates.Note that changes to the scene graph structure (eg, moving/deleting nodes should NOT be done within this callback because traversals stacks will become corrupted. The technique is rather to enable a flag and then do the actual change in the SceneManager::updateGraph() method. ");
	I_Method1(void, setContext, IN, const char *, newvalue,
	          Properties::VIRTUAL,
	          __void__setContext__C5_char_P1,
	          "",
	          "A node can 'belong' to a certain host machine, allowing it to be rendered or behave differently than on other machines.NOTE: the \"NULL\" string means that it belongs to no specific context.NOTE: a scene operating in SERVER_MODE will always create the node, so this feature is only really relevant for clients applications. ");
	I_Method1(void, setText, IN, const char *, s,
	          Properties::NON_VIRTUAL,
	          __void__setText__C5_char_P1,
	          "",
	          "Accepts user-entered string for the node's text. ");
	I_Method1(void, setTextValue, IN, const char *, s,
	          Properties::NON_VIRTUAL,
	          __void__setTextValue__C5_char_P1,
	          "",
	          "Deprecated method (here for backwards compatibility). ");
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
	I_Method2(void, setBox, IN, float, width, IN, float, height,
	          Properties::NON_VIRTUAL,
	          __void__setBox__float__float,
	          "",
	          "Sets the maximum size of the text box. Values of 0 in either dimension means no maximum, so that the box will stretch to fit the text ");
	I_Method1(void, setLineSpacing, IN, float, spacing,
	          Properties::NON_VIRTUAL,
	          __void__setLineSpacing__float,
	          "",
	          "Sets the line spacing, as a percentage of the character height. The default is 0 ");
	I_Method1(void, setAlignment, IN, int, alignment,
	          Properties::NON_VIRTUAL,
	          __void__setAlignment__int,
	          "",
	          "Sets the maximum size of the text box. ");
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
	I_Method1(void, setSingleSided, IN, int, singleSided,
	          Properties::NON_VIRTUAL,
	          __void__setSingleSided__int,
	          "",
	          "Specify whether both sides or only one side of the text is rendered. ie, whether the backface is culled or not. ");
	I_Method0(const char *, getText,
	          Properties::NON_VIRTUAL,
	          __C5_char_P1__getText,
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
	I_Method0(float, getLineSpacing,
	          Properties::NON_VIRTUAL,
	          __float__getLineSpacing,
	          "",
	          "Returns a float indicating the line spacing (as a percentage of character height). ");
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
	I_Method0(int, getSingleSided,
	          Properties::NON_VIRTUAL,
	          __int__getSingleSided,
	          "",
	          "Returns whether the text is drawn single-sided or not. ");
	I_Method0(std::vector< lo_message >, getState,
	          Properties::VIRTUAL,
	          __std_vectorT1_lo_message___getState,
	          "",
	          "For each subclass of ReferencedNode, we override the getState() method to fill the vector with the correct set of methods for this particular node ");
	I_SimpleProperty(int, Alignment, 
	                 0, 
	                 __void__setAlignment__int);
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
	I_SimpleProperty(float, LineSpacing, 
	                 __float__getLineSpacing, 
	                 __void__setLineSpacing__float);
	I_SimpleProperty(float, Margin, 
	                 __float__getMargin, 
	                 __void__setMargin__float);
	I_SimpleProperty(int, SingleSided, 
	                 __int__getSingleSided, 
	                 __void__setSingleSided__int);
	I_SimpleProperty(float, Size, 
	                 __float__getSize, 
	                 __void__setSize__float);
	I_SimpleProperty(std::vector< lo_message >, State, 
	                 __std_vectorT1_lo_message___getState, 
	                 0);
	I_SimpleProperty(const char *, Text, 
	                 __C5_char_P1__getText, 
	                 __void__setText__C5_char_P1);
	I_SimpleProperty(std::string, TextString, 
	                 __std_string__getTextString, 
	                 0);
	I_SimpleProperty(const char *, TextValue, 
	                 0, 
	                 __void__setTextValue__C5_char_P1);
END_REFLECTOR

