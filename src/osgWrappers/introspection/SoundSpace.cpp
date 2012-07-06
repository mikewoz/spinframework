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
#include <SoundSpace.h>
#include <spinUtil.h>

// Must undefine IN and OUT macros defined in Windows headers
#ifdef IN
#undef IN
#endif
#ifdef OUT
#undef OUT
#endif

BEGIN_OBJECT_REFLECTOR(spin::SoundSpace)
	I_DeclaringFile("SoundSpace.h");
	I_BaseType(spin::DSPNode);
	I_Constructor2(IN, spin::SceneManager *, sceneManager, IN, const char *, initID,
	               ____SoundSpace__SceneManager_P1__C5_char_P1,
	               "",
	               "");
	I_Method1(void, setAbsorption, IN, spin::t_floatarg, newval,
	          Properties::NON_VIRTUAL,
	          __void__setAbsorption__t_floatarg,
	          "",
	          "");
	I_Method1(void, setFilterCoef, IN, spin::t_floatarg, newval,
	          Properties::NON_VIRTUAL,
	          __void__setFilterCoef__t_floatarg,
	          "",
	          "");
	I_Method1(void, setTransition, IN, spin::t_floatarg, newval,
	          Properties::NON_VIRTUAL,
	          __void__setTransition__t_floatarg,
	          "",
	          "");
	I_Method0(std::vector< lo_message >, getState,
	          Properties::VIRTUAL,
	          __std_vectorT1_lo_message___getState,
	          "",
	          "For each subclass of ReferencedNode, we override the getState() method to fill the vector with the correct set of methods for this particular node ");
	I_SimpleProperty(spin::t_floatarg, Absorption, 
	                 0, 
	                 __void__setAbsorption__t_floatarg);
	I_SimpleProperty(spin::t_floatarg, FilterCoef, 
	                 0, 
	                 __void__setFilterCoef__t_floatarg);
	I_SimpleProperty(std::vector< lo_message >, State, 
	                 __std_vectorT1_lo_message___getState, 
	                 0);
	I_SimpleProperty(spin::t_floatarg, Transition, 
	                 0, 
	                 __void__setTransition__t_floatarg);
	I_PublicMemberProperty(spin::t_float, absorption);
	I_PublicMemberProperty(spin::t_float, filterCoef);
	I_PublicMemberProperty(spin::t_float, transition);
END_REFLECTOR

