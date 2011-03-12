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

#include <MediaManager.h>

// Must undefine IN and OUT macros defined in Windows headers
#ifdef IN
#undef IN
#endif
#ifdef OUT
#undef OUT
#endif

BEGIN_VALUE_REFLECTOR(MediaManager)
	I_DeclaringFile("MediaManager.h");
	I_Constructor1(IN, const std::string &, dataPath,
	               Properties::NON_EXPLICIT,
	               ____MediaManager__C5_std_string_R1,
	               "",
	               "");
	I_Method0(std::string, getDataPath,
	          Properties::NON_VIRTUAL,
	          __std_string__getDataPath,
	          "",
	          "");
	I_Method1(std::string, getImagePath, IN, const std::string &, s,
	          Properties::NON_VIRTUAL,
	          __std_string__getImagePath__C5_std_string_R1,
	          "",
	          "");
	I_Method1(std::string, getModelPath, IN, const std::string &, s,
	          Properties::NON_VIRTUAL,
	          __std_string__getModelPath__C5_std_string_R1,
	          "",
	          "");
	I_Method1(std::string, getSoundPath, IN, const std::string &, s,
	          Properties::NON_VIRTUAL,
	          __std_string__getSoundPath__C5_std_string_R1,
	          "",
	          "");
	I_Method1(std::string, getImagePath, IN, int, id,
	          Properties::NON_VIRTUAL,
	          __std_string__getImagePath__int,
	          "",
	          "");
	I_Method1(std::string, getModelPath, IN, int, id,
	          Properties::NON_VIRTUAL,
	          __std_string__getModelPath__int,
	          "",
	          "");
	I_Method1(std::string, getSoundPath, IN, int, id,
	          Properties::NON_VIRTUAL,
	          __std_string__getSoundPath__int,
	          "",
	          "");
	I_Method1(std::string, getImageName, IN, int, id,
	          Properties::NON_VIRTUAL,
	          __std_string__getImageName__int,
	          "",
	          "");
	I_Method1(std::string, getModelName, IN, int, id,
	          Properties::NON_VIRTUAL,
	          __std_string__getModelName__int,
	          "",
	          "");
	I_Method1(std::string, getSoundName, IN, int, id,
	          Properties::NON_VIRTUAL,
	          __std_string__getSoundName__int,
	          "",
	          "");
	I_SimpleProperty(std::string, DataPath, 
	                 __std_string__getDataPath, 
	                 0);
END_REFLECTOR

