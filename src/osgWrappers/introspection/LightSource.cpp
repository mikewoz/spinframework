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

#include <LightSource.h>
#include <SceneManager.h>

// Must undefine IN and OUT macros defined in Windows headers
#ifdef IN
#undef IN
#endif
#ifdef OUT
#undef OUT
#endif

BEGIN_OBJECT_REFLECTOR(spin::LightSource)
	I_DeclaringFile("LightSource.h");
	I_BaseType(spin::GroupNode);
	I_Constructor2(IN, spin::SceneManager *, sceneManager, IN, const char *, initID,
	               ____LightSource__SceneManager_P1__C5_char_P1,
	               "",
	               "");
	I_Method1(void, setVisible, IN, int, visibilityFlag,
	          Properties::NON_VIRTUAL,
	          __void__setVisible__int,
	          "",
	          "Set whether the light source is visible. ");
	I_Method1(void, setCutoff, IN, float, cutoff,
	          Properties::NON_VIRTUAL,
	          __void__setCutoff__float,
	          "",
	          "");
	I_Method1(void, setExponent, IN, float, exponent,
	          Properties::NON_VIRTUAL,
	          __void__setExponent__float,
	          "",
	          "");
	I_Method1(void, setAttenuation, IN, float, attenuation,
	          Properties::NON_VIRTUAL,
	          __void__setAttenuation__float,
	          "",
	          "");
	I_Method4(void, setAmbient, IN, float, red, IN, float, green, IN, float, blue, IN, float, alpha,
	          Properties::NON_VIRTUAL,
	          __void__setAmbient__float__float__float__float,
	          "",
	          "Sets the ambient value for the light source in RGBA. ");
	I_Method4(void, setDiffuse, IN, float, red, IN, float, green, IN, float, blue, IN, float, alpha,
	          Properties::NON_VIRTUAL,
	          __void__setDiffuse__float__float__float__float,
	          "",
	          "Sets the diffuse value for the light source in RGBA. ");
	I_Method4(void, setSpecular, IN, float, red, IN, float, green, IN, float, blue, IN, float, alpha,
	          Properties::NON_VIRTUAL,
	          __void__setSpecular__float__float__float__float,
	          "",
	          "Sets the specular value for the light source in RGBA. ");
	I_Method0(int, getVisible,
	          Properties::NON_VIRTUAL,
	          __int__getVisible,
	          "",
	          "Returns a boolean indicating whether the light source is visible. ");
	I_Method0(float, getCutoff,
	          Properties::NON_VIRTUAL,
	          __float__getCutoff,
	          "",
	          "");
	I_Method0(float, getExponent,
	          Properties::NON_VIRTUAL,
	          __float__getExponent,
	          "",
	          "");
	I_Method0(float, getAttenuation,
	          Properties::NON_VIRTUAL,
	          __float__getAttenuation,
	          "",
	          "");
	I_Method0(osg::Vec4, getAmbient,
	          Properties::NON_VIRTUAL,
	          __osg_Vec4__getAmbient,
	          "",
	          "Returns the ambient value of the light source in RGBA. ");
	I_Method0(osg::Vec4, getDiffuse,
	          Properties::NON_VIRTUAL,
	          __osg_Vec4__getDiffuse,
	          "",
	          "Returns the diffuse value of the light source in RGBA. ");
	I_Method0(osg::Vec4, getSpecular,
	          Properties::NON_VIRTUAL,
	          __osg_Vec4__getSpecular,
	          "",
	          "Returns the specular value of the light source in RGBA. ");
	I_Method0(std::vector< lo_message >, getState,
	          Properties::VIRTUAL,
	          __std_vectorT1_lo_message___getState,
	          "",
	          "For each subclass of ReferencedNode, we override the getState() method to fill the vector with the correct set of methods for this particular node ");
	I_SimpleProperty(osg::Vec4, Ambient, 
	                 __osg_Vec4__getAmbient, 
	                 0);
	I_SimpleProperty(float, Attenuation, 
	                 __float__getAttenuation, 
	                 __void__setAttenuation__float);
	I_SimpleProperty(float, Cutoff, 
	                 __float__getCutoff, 
	                 __void__setCutoff__float);
	I_SimpleProperty(osg::Vec4, Diffuse, 
	                 __osg_Vec4__getDiffuse, 
	                 0);
	I_SimpleProperty(float, Exponent, 
	                 __float__getExponent, 
	                 __void__setExponent__float);
	I_SimpleProperty(osg::Vec4, Specular, 
	                 __osg_Vec4__getSpecular, 
	                 0);
	I_SimpleProperty(std::vector< lo_message >, State, 
	                 __std_vectorT1_lo_message___getState, 
	                 0);
	I_SimpleProperty(int, Visible, 
	                 __int__getVisible, 
	                 __void__setVisible__int);
END_REFLECTOR

