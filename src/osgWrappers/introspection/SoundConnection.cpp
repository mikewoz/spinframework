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

#include <DSPNode.h>
#include <SceneManager.h>
#include <SoundConnection.h>

// Must undefine IN and OUT macros defined in Windows headers
#ifdef IN
#undef IN
#endif
#ifdef OUT
#undef OUT
#endif

BEGIN_VALUE_REFLECTOR(spin::SoundConnection)
	I_DeclaringFile("SoundConnection.h");
	I_Constructor3(IN, spin::SceneManager *, s, IN, osg::ref_ptr< spin::DSPNode >, src, IN, osg::ref_ptr< spin::DSPNode >, snk,
	               ____SoundConnection__SceneManager_P1__osg_ref_ptrT1_DSPNode___osg_ref_ptrT1_DSPNode_,
	               "",
	               "");
	I_Method0(std::string, getID,
	          Properties::NON_VIRTUAL,
	          __std_string__getID,
	          "",
	          "");
	I_Method1(void, setThru, IN, int, newvalue,
	          Properties::NON_VIRTUAL,
	          __void__setThru__int,
	          "",
	          "");
	I_Method1(void, setDistanceEffect, IN, float, newvalue,
	          Properties::NON_VIRTUAL,
	          __void__setDistanceEffect__float,
	          "",
	          "");
	I_Method1(void, setRolloffEffect, IN, float, newvalue,
	          Properties::NON_VIRTUAL,
	          __void__setRolloffEffect__float,
	          "",
	          "");
	I_Method1(void, setDopplerEffect, IN, float, newvalue,
	          Properties::NON_VIRTUAL,
	          __void__setDopplerEffect__float,
	          "",
	          "");
	I_Method1(void, setDiffractionEffect, IN, float, newvalue,
	          Properties::NON_VIRTUAL,
	          __void__setDiffractionEffect__float,
	          "",
	          "");
	I_Method0(int, getThru,
	          Properties::NON_VIRTUAL,
	          __int__getThru,
	          "",
	          "");
	I_Method0(float, getDistanceEffect,
	          Properties::NON_VIRTUAL,
	          __float__getDistanceEffect,
	          "",
	          "");
	I_Method0(float, getRolloffEffect,
	          Properties::NON_VIRTUAL,
	          __float__getRolloffEffect,
	          "",
	          "");
	I_Method0(float, getDopplerEffect,
	          Properties::NON_VIRTUAL,
	          __float__getDopplerEffect,
	          "",
	          "");
	I_Method0(float, getDiffractionEffect,
	          Properties::NON_VIRTUAL,
	          __float__getDiffractionEffect,
	          "",
	          "");
	I_Method0(void, debug,
	          Properties::NON_VIRTUAL,
	          __void__debug,
	          "",
	          "");
	I_Method0(std::vector< lo_message >, getState,
	          Properties::VIRTUAL,
	          __std_vectorT1_lo_message___getState,
	          "",
	          "");
	I_Method0(void, stateDump,
	          Properties::VIRTUAL,
	          __void__stateDump,
	          "",
	          "");
	I_Method1(void, stateDump, IN, lo_address, addr,
	          Properties::VIRTUAL,
	          __void__stateDump__lo_address,
	          "",
	          "");
	I_SimpleProperty(float, DiffractionEffect, 
	                 __float__getDiffractionEffect, 
	                 __void__setDiffractionEffect__float);
	I_SimpleProperty(float, DistanceEffect, 
	                 __float__getDistanceEffect, 
	                 __void__setDistanceEffect__float);
	I_SimpleProperty(float, DopplerEffect, 
	                 __float__getDopplerEffect, 
	                 __void__setDopplerEffect__float);
	I_SimpleProperty(std::string, ID, 
	                 __std_string__getID, 
	                 0);
	I_SimpleProperty(float, RolloffEffect, 
	                 __float__getRolloffEffect, 
	                 __void__setRolloffEffect__float);
	I_SimpleProperty(std::vector< lo_message >, State, 
	                 __std_vectorT1_lo_message___getState, 
	                 0);
	I_SimpleProperty(int, Thru, 
	                 __int__getThru, 
	                 __void__setThru__int);
	I_PublicMemberProperty(spin::DSPNode *, source);
	I_PublicMemberProperty(spin::DSPNode *, sink);
END_REFLECTOR

BEGIN_ENUM_REFLECTOR(spin::connectionType)
	I_DeclaringFile("SoundConnection.h");
	I_EnumLabel(spin::NORMAL);
	I_EnumLabel(spin::NODE_TO_SPACE);
	I_EnumLabel(spin::SPACE_TO_NODE);
END_REFLECTOR

