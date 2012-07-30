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

BEGIN_ENUM_REFLECTOR(spin::TextNode::DrawMode)
	I_DeclaringFile("TextNode.h");
	I_EnumLabel(spin::TextNode::GLYPH);
	I_EnumLabel(spin::TextNode::TEXT3D);
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
	I_Constructor2(IN, spin::SceneManager *, sceneManager, IN, const char *, initID,
	               ____TextNode__SceneManager_P1__C5_char_P1,
	               "",
	               "");
	I_Method1(void, callbackUpdate, IN, osg::NodeVisitor *, nv,
	          Properties::VIRTUAL,
	          __void__callbackUpdate__osg_NodeVisitor_P1,
	          "",
	          "For nodes that require regular programmatic control, there is a callback that is evaluated with every refresh. This function can thus be used for animations, or any other periodic updates.Note that changes to the scene graph structure (eg, moving/deleting nodes should NOT be done within this callback because traversals stacks will become corrupted. The technique is rather to enable a flag and then do the actual change in the SceneManager::updateGraph() method. ");
	I_Method1(void, setContext, IN, const char *, newvalue,
	          Properties::VIRTUAL,
	          __void__setContext__C5_char_P1,
	          "",
	          "A node can 'belong' to a certain host machine, allowing it to be rendered or behave differently than on other machines.NOTE: the \"NULL\" string means that it belongs to no specific context.NOTE: a scene operating in SERVER_MODE will always create the node, so this feature is only really relevant for clients applications. ");
	I_Method1(void, setDrawMode, IN, spin::TextNode::DrawMode, mode,
	          Properties::NON_VIRTUAL,
	          __void__setDrawMode__DrawMode,
	          "",
	          "Choose whether the text is drawn in 2D glyphs, or as 3D geometry. ");
	I_Method0(int, getDrawMode,
	          Properties::NON_VIRTUAL,
	          __int__getDrawMode,
	          "",
	          "");
	I_Method1(void, setText, IN, const char *, s,
	          Properties::NON_VIRTUAL,
	          __void__setText__C5_char_P1,
	          "",
	          "Accepts user-entered string for the node's text. ");
	I_Method0(const char *, getText,
	          Properties::NON_VIRTUAL,
	          __C5_char_P1__getText,
	          "",
	          "");
	I_Method0(std::string, getTextString,
	          Properties::NON_VIRTUAL,
	          __std_string__getTextString,
	          "",
	          "");
	I_Method1(void, setTextValue, IN, const char *, s,
	          Properties::NON_VIRTUAL,
	          __void__setTextValue__C5_char_P1,
	          "",
	          "Deprecated method (here for backwards compatibility). ");
	I_Method1(void, setRepetitions, IN, int, repetitions,
	          Properties::NON_VIRTUAL,
	          __void__setRepetitions__int,
	          "",
	          "Set the number of times the text should repeat ");
	I_Method0(int, getRepetitions,
	          Properties::NON_VIRTUAL,
	          __int__getRepetitions,
	          "",
	          "");
	I_Method1(void, setFont, IN, const char *, s,
	          Properties::NON_VIRTUAL,
	          __void__setFont__C5_char_P1,
	          "",
	          "Sets the font for the text associated with this node. ");
	I_Method0(const char *, getFont,
	          Properties::NON_VIRTUAL,
	          __C5_char_P1__getFont,
	          "",
	          "");
	I_Method1(void, setFontResolution, IN, int, resolution,
	          Properties::NON_VIRTUAL,
	          __void__setFontResolution__int,
	          "",
	          "Set the font resolution. eg, 128 will produce 128x128 textures ");
	I_Method0(int, getFontResolution,
	          Properties::NON_VIRTUAL,
	          __int__getFontResolution,
	          "",
	          "");
	I_Method1(void, setCharacterSize, IN, float, s,
	          Properties::NON_VIRTUAL,
	          __void__setCharacterSize__float,
	          "",
	          "Sets the size of text characters (in local coordinate system units) ");
	I_Method0(float, getCharacterSize,
	          Properties::NON_VIRTUAL,
	          __float__getCharacterSize,
	          "",
	          "");
	I_Method1(void, setThickness, IN, float, thickness,
	          Properties::NON_VIRTUAL,
	          __void__setThickness__float,
	          "",
	          "Set the thickness of 3D text ");
	I_Method0(float, getThickness,
	          Properties::NON_VIRTUAL,
	          __float__getThickness,
	          "",
	          "");
	I_Method2(void, setBoxSize, IN, float, width, IN, float, height,
	          Properties::NON_VIRTUAL,
	          __void__setBoxSize__float__float,
	          "",
	          "Sets the maximum size of the text box. Values of 0 in either dimension means no maximum, so that the box will stretch to fit the text ");
	I_Method1(void, setLineSpacing, IN, float, spacing,
	          Properties::NON_VIRTUAL,
	          __void__setLineSpacing__float,
	          "",
	          "Sets the line spacing, as a percentage of the character height. The default is 0 ");
	I_Method0(float, getLineSpacing,
	          Properties::NON_VIRTUAL,
	          __float__getLineSpacing,
	          "",
	          "");
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
	I_Method0(osg::Vec4, getColor,
	          Properties::NON_VIRTUAL,
	          __osg_Vec4__getColor,
	          "",
	          "");
	I_Method4(void, setBgColor, IN, float, red, IN, float, green, IN, float, blue, IN, float, alpha,
	          Properties::NON_VIRTUAL,
	          __void__setBgColor__float__float__float__float,
	          "",
	          "Sets the background color for this node. ");
	I_Method0(osg::Vec4, getBgColor,
	          Properties::NON_VIRTUAL,
	          __osg_Vec4__getBgColor,
	          "",
	          "");
	I_Method1(void, setMargin, IN, float, margin,
	          Properties::NON_VIRTUAL,
	          __void__setMargin__float,
	          "",
	          "Sets the margins for the text associated to this node. ");
	I_Method0(float, getMargin,
	          Properties::NON_VIRTUAL,
	          __float__getMargin,
	          "",
	          "");
	I_Method1(void, setBillboard, IN, spin::TextNode::billboardType, t,
	          Properties::NON_VIRTUAL,
	          __void__setBillboard__billboardType,
	          "",
	          "Sets the type of billboarding asigned to this node (drawn from the enum billboardType). ");
	I_Method0(int, getBillboard,
	          Properties::NON_VIRTUAL,
	          __int__getBillboard,
	          "",
	          "");
	I_Method1(void, setDecoration, IN, spin::TextNode::decorationType, t,
	          Properties::NON_VIRTUAL,
	          __void__setDecoration__decorationType,
	          "",
	          "Sets the shadowing or outline type for this text node (drawn from the decorationType enum). ");
	I_Method0(int, getDecoration,
	          Properties::NON_VIRTUAL,
	          __int__getDecoration,
	          "",
	          "");
	I_Method1(void, setBackground, IN, spin::TextNode::backgroundType, t,
	          Properties::NON_VIRTUAL,
	          __void__setBackground__backgroundType,
	          "",
	          "Sets a background type for the text box (drawn from the backgroundType enum). ");
	I_Method0(int, getBackround,
	          Properties::NON_VIRTUAL,
	          __int__getBackround,
	          "",
	          "");
	I_Method1(void, setSingleSided, IN, int, singleSided,
	          Properties::NON_VIRTUAL,
	          __void__setSingleSided__int,
	          "",
	          "Specify whether both sides or only one side of the text is rendered. ie, whether the backface is culled or not. ");
	I_Method0(int, getSingleSided,
	          Properties::NON_VIRTUAL,
	          __int__getSingleSided,
	          "",
	          "");
	I_Method1(void, setLighting, IN, int, lighting,
	          Properties::NON_VIRTUAL,
	          __void__setLighting__int,
	          "",
	          "Specify whether both sides or only one side of the text is rendered. ie, whether the backface is culled or not. ");
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
	I_SimpleProperty(float, CharacterSize, 
	                 __float__getCharacterSize, 
	                 __void__setCharacterSize__float);
	I_SimpleProperty(osg::Vec4, Color, 
	                 __osg_Vec4__getColor, 
	                 0);
	I_SimpleProperty(const char *, Context, 
	                 0, 
	                 __void__setContext__C5_char_P1);
	I_SimpleProperty(int, Decoration, 
	                 __int__getDecoration, 
	                 0);
	I_SimpleProperty(int, DrawMode, 
	                 __int__getDrawMode, 
	                 0);
	I_SimpleProperty(const char *, Font, 
	                 __C5_char_P1__getFont, 
	                 __void__setFont__C5_char_P1);
	I_SimpleProperty(int, FontResolution, 
	                 __int__getFontResolution, 
	                 __void__setFontResolution__int);
	I_SimpleProperty(int, Lighting, 
	                 __int__getLighting, 
	                 __void__setLighting__int);
	I_SimpleProperty(float, LineSpacing, 
	                 __float__getLineSpacing, 
	                 __void__setLineSpacing__float);
	I_SimpleProperty(float, Margin, 
	                 __float__getMargin, 
	                 __void__setMargin__float);
	I_SimpleProperty(int, Repetitions, 
	                 __int__getRepetitions, 
	                 __void__setRepetitions__int);
	I_SimpleProperty(int, SingleSided, 
	                 __int__getSingleSided, 
	                 __void__setSingleSided__int);
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
	I_SimpleProperty(float, Thickness, 
	                 __float__getThickness, 
	                 __void__setThickness__float);
END_REFLECTOR

