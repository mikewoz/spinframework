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

#include <DraggerTrackball.h>

// Must undefine IN and OUT macros defined in Windows headers
#ifdef IN
#undef IN
#endif
#ifdef OUT
#undef OUT
#endif

BEGIN_OBJECT_REFLECTOR(osgManipulator::DraggerTrackball)
	I_DeclaringFile("DraggerTrackball.h");
	I_ConstructorWithDefaults1(IN, bool, useAutoTransform, false,
	                           Properties::NON_EXPLICIT,
	                           ____DraggerTrackball__bool,
	                           "",
	                           "");
	I_Method2(, META_OSGMANIPULATOR_Object, IN, osgManipulator, x, IN, osgManipulator::DraggerTrackball, x,
	          Properties::NON_VIRTUAL,
	          ____META_OSGMANIPULATOR_Object__osgManipulator__DraggerTrackball,
	          "",
	          "Setup default geometry for dragger. ");
END_REFLECTOR

