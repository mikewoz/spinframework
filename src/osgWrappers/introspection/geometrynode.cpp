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

#include <geometrynode.h>
#include <scenemanager.h>

// Must undefine IN and OUT macros defined in Windows headers
#ifdef IN
#undef IN
#endif
#ifdef OUT
#undef OUT
#endif

BEGIN_OBJECT_REFLECTOR(spin::GeometryNode)
	I_DeclaringFile("geometrynode.h");
	I_BaseType(spin::GroupNode);
	I_Constructor2(IN, spin::SceneManager *, sceneManager, IN, const char *, initID,
	               ____GeometryNode__SceneManager_P1__C5_char_P1,
	               "",
	               "");
	I_Method1(void, callbackUpdate, IN, osg::NodeVisitor *, nv,
	          Properties::VIRTUAL,
	          __void__callbackUpdate__osg_NodeVisitor_P1,
	          "",
	          "For nodes that require regular programmatic control, there is a callback that is evaluated with every refresh. This function can thus be used for animations, or any other periodic updates.Note that changes to the scene graph structure (eg, moving/deleting nodes should NOT be done within this callback because traversals stacks will become corrupted. The technique is rather to enable a flag and then do the actual change in the SceneManager::updateGraph() method. ");
	I_Method0(void, updateStateSet,
	          Properties::VIRTUAL,
	          __void__updateStateSet,
	          "",
	          "This method actually applies the stateset to the subgraph, replacing any existing stateset with this one. The setStateSet and setStateSetFromFile methods just set the stateset_ symbol, while updateStateSet does the actual work.Override this method in subclasses in order to change how stateset should be applied. For example, to which node in the subgraph it should be attached, or whether it should be merged with the existing stateset (rather than merged).By default it is applied to the mainTransform_. ");
	I_Method1(void, setNumVertices, IN, int, i,
	          Properties::NON_VIRTUAL,
	          __void__setNumVertices__int,
	          "",
	          "Sets the number of vertices that this geometry is supposed to have.This should be a multiple of 3 if you want to draw using GL_TRIANGLES or amultiple of 4 if you want to draw using GL_QUADS. Anything else will draw using GL_LINES.If you grow the number of vertices, all new vertices will be created with default values: position of (0,0,0), full white color, and texcoords of (0,0). ");
	I_Method4(void, setVertex, IN, int, index, IN, float, x, IN, float, y, IN, float, z,
	          Properties::NON_VIRTUAL,
	          __void__setVertex__int__float__float__float,
	          "",
	          "Update the position of one vertex in the geometry, using an index. ");
	I_Method5(void, setColor, IN, int, index, IN, float, red, IN, float, green, IN, float, blue, IN, float, alpha,
	          Properties::NON_VIRTUAL,
	          __void__setColor__int__float__float__float__float,
	          "",
	          "Update the color of one vertex in the geometry, using an index. ");
	I_Method3(void, setTexCoord, IN, int, index, IN, float, x, IN, float, y,
	          Properties::NON_VIRTUAL,
	          __void__setTexCoord__int__float__float,
	          "",
	          "Update the texture coord of one vertex in the geometry, using an index. ");
	I_Method1(void, setSingleSided, IN, int, singleSided,
	          Properties::NON_VIRTUAL,
	          __void__setSingleSided__int,
	          "",
	          "Specify whether both sides or only one side of the shape is rendered. ie, whether the backface is culled or not. ");
	I_Method0(int, getSingleSided,
	          Properties::NON_VIRTUAL,
	          __int__getSingleSided,
	          "",
	          "Returns whether both sides of the shape are being rendered or if the backface is being culled. ");
	I_Method0(std::vector< lo_message >, getState,
	          Properties::VIRTUAL,
	          __std_vectorT1_lo_message___getState,
	          "",
	          "For each subclass of ReferencedNode, we override the getState() method to fill the vector with the correct set of methods for this particular node ");
	I_SimpleProperty(int, NumVertices, 
	                 0, 
	                 __void__setNumVertices__int);
	I_SimpleProperty(int, SingleSided, 
	                 __int__getSingleSided, 
	                 __void__setSingleSided__int);
	I_SimpleProperty(std::vector< lo_message >, State, 
	                 __std_vectorT1_lo_message___getState, 
	                 0);
END_REFLECTOR
