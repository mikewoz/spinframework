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

#include <PointCloud.h>
#include <SceneManager.h>

// Must undefine IN and OUT macros defined in Windows headers
#ifdef IN
#undef IN
#endif
#ifdef OUT
#undef OUT
#endif

BEGIN_ENUM_REFLECTOR(spin::PointCloud::DrawMode)
	I_DeclaringFile("PointCloud.h");
	I_EnumLabel(spin::PointCloud::NONE);
	I_EnumLabel(spin::PointCloud::POINTS);
	I_EnumLabel(spin::PointCloud::LINES);
	I_EnumLabel(spin::PointCloud::LINE_STRIP);
	I_EnumLabel(spin::PointCloud::LINE_LOOP);
	I_EnumLabel(spin::PointCloud::TRIANGLES);
	I_EnumLabel(spin::PointCloud::TRIANGLE_STRIP);
	I_EnumLabel(spin::PointCloud::TRIANGLE_FAN);
	I_EnumLabel(spin::PointCloud::QUADS);
	I_EnumLabel(spin::PointCloud::QUAD_STRIP);
	I_EnumLabel(spin::PointCloud::POLYGON);
	I_EnumLabel(spin::PointCloud::LIGHTPOINTS);
	I_EnumLabel(spin::PointCloud::BOXES);
	I_EnumLabel(spin::PointCloud::CUSTOM);
END_REFLECTOR

BEGIN_ENUM_REFLECTOR(spin::PointCloud::ColorMode)
	I_DeclaringFile("PointCloud.h");
	I_EnumLabel(spin::PointCloud::NORMAL);
	I_EnumLabel(spin::PointCloud::OVERRIDE);
END_REFLECTOR

BEGIN_OBJECT_REFLECTOR(spin::PointCloud)
	I_DeclaringFile("PointCloud.h");
	I_BaseType(spin::GroupNode);
	I_Constructor2(IN, spin::SceneManager *, sceneManager, IN, const char *, initID,
	               ____PointCloud__SceneManager_P1__C5_char_P1,
	               "",
	               "");
	I_Method0(void, debug,
	          Properties::VIRTUAL,
	          __void__debug,
	          "",
	          "Print debug information about the node to standard out (when running in console mode). It may be possible to redirect this to a text box for GUI logs. ");
	I_Method1(void, callbackUpdate, IN, osg::NodeVisitor *, nv,
	          Properties::VIRTUAL,
	          __void__callbackUpdate__osg_NodeVisitor_P1,
	          "",
	          "For nodes that require regular programmatic control, there is a callback that is evaluated with every refresh. This function can thus be used for animations, or any other periodic updates.Note that changes to the scene graph structure (eg, moving/deleting nodes should NOT be done within this callback because traversals stacks will become corrupted. The technique is rather to enable a flag and then do the actual change in the SceneManager::updateGraph() method. ");
	I_Method1(void, setURI, IN, const char *, filename,
	          Properties::NON_VIRTUAL,
	          __void__setURI__C5_char_P1,
	          "",
	          "");
	I_Method1(void, grabberCallback, IN, const pcl::PointCloud< pcl::PointXYZRGBA >::ConstPtr &, cloud,
	          Properties::NON_VIRTUAL,
	          __void__grabberCallback__C5_pcl_PointCloudT1_pcl_PointXYZRGBA__ConstPtr_R1,
	          "",
	          "");
	I_Method1(void, applyFilters, IN, const pcl::PointCloud< pcl::PointXYZRGBA >::ConstPtr &, rawCloud,
	          Properties::NON_VIRTUAL,
	          __void__applyFilters__C5_pcl_PointCloudT1_pcl_PointXYZRGBA__ConstPtr_R1,
	          "",
	          "");
	I_Method1(osg::Vec3, getPos, IN, unsigned int, i,
	          Properties::NON_VIRTUAL,
	          __osg_Vec3__getPos__unsigned_int,
	          "",
	          "");
	I_Method1(osg::Vec4f, getColor, IN, unsigned int, i,
	          Properties::NON_VIRTUAL,
	          __osg_Vec4f__getColor__unsigned_int,
	          "",
	          "");
	I_Method0(void, updatePoints,
	          Properties::VIRTUAL,
	          __void__updatePoints,
	          "",
	          "");
	I_Method0(void, draw,
	          Properties::VIRTUAL,
	          __void__draw,
	          "",
	          "");
	I_Method1(void, setCustomNode, IN, const char *, nodeID,
	          Properties::NON_VIRTUAL,
	          __void__setCustomNode__C5_char_P1,
	          "",
	          "");
	I_Method1(void, setDrawMode, IN, spin::PointCloud::DrawMode, mode,
	          Properties::NON_VIRTUAL,
	          __void__setDrawMode__DrawMode,
	          "",
	          "");
	I_Method1(void, setSpacing, IN, float, spacing,
	          Properties::NON_VIRTUAL,
	          __void__setSpacing__float,
	          "",
	          "");
	I_Method1(void, setRandomCoeff, IN, float, randomCoeff,
	          Properties::NON_VIRTUAL,
	          __void__setRandomCoeff__float,
	          "",
	          "");
	I_Method1(void, setPointSize, IN, float, pointsize,
	          Properties::NON_VIRTUAL,
	          __void__setPointSize__float,
	          "",
	          "");
	I_Method4(void, setColor, IN, float, red, IN, float, green, IN, float, blue, IN, float, alpha,
	          Properties::NON_VIRTUAL,
	          __void__setColor__float__float__float__float,
	          "",
	          "");
	I_Method1(void, setColorMode, IN, spin::PointCloud::ColorMode, mode,
	          Properties::NON_VIRTUAL,
	          __void__setColorMode__ColorMode,
	          "",
	          "");
	I_Method1(void, setVoxelSize, IN, float, voxelSize,
	          Properties::NON_VIRTUAL,
	          __void__setVoxelSize__float,
	          "",
	          "Set the voxelSize for the pcl::VoxelGrid filter, which downsamples points to the nearest voxel in a 3D voxel grid. Think about a voxel grid as a set of tiny 3D boxes in space. param voxelSize in metres (default is 0.01, ie, 1cm). A value of 0 will disable the VoxelGrid filter.  ");
	I_Method2(void, setDistCrop, IN, float, min, IN, float, max,
	          Properties::NON_VIRTUAL,
	          __void__setDistCrop__float__float,
	          "",
	          "Set the minimum and maximum distance of valid points (in metres) ");
	I_Method0(const char *, getCustomNode,
	          Properties::NON_VIRTUAL,
	          __C5_char_P1__getCustomNode,
	          "",
	          "");
	I_Method0(int, getDrawMode,
	          Properties::NON_VIRTUAL,
	          __int__getDrawMode,
	          "",
	          "");
	I_Method0(float, getSpacing,
	          Properties::NON_VIRTUAL,
	          __float__getSpacing,
	          "",
	          "");
	I_Method0(float, getRandomCoeff,
	          Properties::NON_VIRTUAL,
	          __float__getRandomCoeff,
	          "",
	          "");
	I_Method0(float, getPointSize,
	          Properties::NON_VIRTUAL,
	          __float__getPointSize,
	          "",
	          "");
	I_Method0(osg::Vec4, getColor,
	          Properties::NON_VIRTUAL,
	          __osg_Vec4__getColor,
	          "",
	          "");
	I_Method0(int, getColorMode,
	          Properties::NON_VIRTUAL,
	          __int__getColorMode,
	          "",
	          "");
	I_Method0(float, getFilterSize,
	          Properties::NON_VIRTUAL,
	          __float__getFilterSize,
	          "",
	          "");
	I_Method0(osg::Vec2, getDistCrop,
	          Properties::NON_VIRTUAL,
	          __osg_Vec2__getDistCrop,
	          "",
	          "");
	I_Method0(std::vector< lo_message >, getState,
	          Properties::VIRTUAL,
	          __std_vectorT1_lo_message___getState,
	          "",
	          "For each subclass of ReferencedNode, we override the getState() method to fill the vector with the correct set of methods for this particular node ");
	I_SimpleProperty(osg::Vec4, Color, 
	                 __osg_Vec4__getColor, 
	                 0);
	I_SimpleProperty(int, ColorMode, 
	                 __int__getColorMode, 
	                 0);
	I_SimpleProperty(const char *, CustomNode, 
	                 __C5_char_P1__getCustomNode, 
	                 __void__setCustomNode__C5_char_P1);
	I_SimpleProperty(osg::Vec2, DistCrop, 
	                 __osg_Vec2__getDistCrop, 
	                 0);
	I_SimpleProperty(int, DrawMode, 
	                 __int__getDrawMode, 
	                 0);
	I_SimpleProperty(float, FilterSize, 
	                 __float__getFilterSize, 
	                 0);
	I_SimpleProperty(float, PointSize, 
	                 __float__getPointSize, 
	                 __void__setPointSize__float);
	I_SimpleProperty(float, RandomCoeff, 
	                 __float__getRandomCoeff, 
	                 __void__setRandomCoeff__float);
	I_SimpleProperty(float, Spacing, 
	                 __float__getSpacing, 
	                 __void__setSpacing__float);
	I_SimpleProperty(std::vector< lo_message >, State, 
	                 __std_vectorT1_lo_message___getState, 
	                 0);
	I_SimpleProperty(const char *, URI, 
	                 0, 
	                 __void__setURI__C5_char_P1);
	I_SimpleProperty(float, VoxelSize, 
	                 0, 
	                 __void__setVoxelSize__float);
END_REFLECTOR

