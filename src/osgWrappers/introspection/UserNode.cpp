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
#include <UserNode.h>

// Must undefine IN and OUT macros defined in Windows headers
#ifdef IN
#undef IN
#endif
#ifdef OUT
#undef OUT
#endif

BEGIN_OBJECT_REFLECTOR(spin::UserNode)
	I_DeclaringFile("UserNode.h");
	I_BaseType(spin::ConstraintsNode);
	I_Constructor2(IN, spin::SceneManager *, sceneManager, IN, char *, initID,
	               ____UserNode__SceneManager_P1__char_P1,
	               "",
	               "");
	I_Method0(void, callbackUpdate,
	          Properties::VIRTUAL,
	          __void__callbackUpdate,
	          "",
	          "The UserNode needs an update callback to check if ping messages are still being received. If not, the node and it's subgraph should be removed. Please note that if the user node NEVER sends a ping, not even once, then it will be excluded from this obligation. ");
	I_Method1(void, updateNodePath, IN, bool, updateChildren,
	          Properties::VIRTUAL,
	          __void__updateNodePath__bool,
	          "",
	          "The UserNode is used by OSG's NodeTrackerManipulator to position a camera for the user. However, NodeTrackerManipulator doesn't check if the nodepath has changed, so we override updateNodePath() and set a nodepathUpdate flag for the manipulator to see. ");
	I_Method1(void, setDescription, IN, const char *, s,
	          Properties::NON_VIRTUAL,
	          __void__setDescription__C5_char_P1,
	          "",
	          "Sets a user-entered description for the node. ");
	I_Method0(const char *, getDescription,
	          Properties::NON_VIRTUAL,
	          __C5_char_P1__getDescription,
	          "",
	          "Returns a string containing the user-entered description for this node. ");
	I_Method0(osg::PositionAttitudeTransform *, getCameraAttachmentNode,
	          Properties::NON_VIRTUAL,
	          __osg_PositionAttitudeTransform_P1__getCameraAttachmentNode,
	          "",
	          "This is where you attach cameras for the user.The standard ViewerManipulator for SPIN derives from OSG's NodeTrackerManipulator, which tracks (points to) the center of a subgraph's bounding sphere. We want to keep that bound empty so that we may effectively place the camera right at the UserNode's location. If we attach geometry under this, the camera will point at the center of that geometry instead. ");
	I_Method0(osg::PositionAttitudeTransform *, getCameraOffsetNode,
	          Properties::NON_VIRTUAL,
	          __osg_PositionAttitudeTransform_P1__getCameraOffsetNode,
	          "",
	          "");
	I_Method3(void, setCameraOffset, IN, float, x, IN, float, y, IN, float, z,
	          Properties::NON_VIRTUAL,
	          __void__setCameraOffset__float__float__float,
	          "",
	          "Set the camera offset (from the UserNode's local origin). The default is (0,0,0), meaning that the camera position is exactly aligned with the UserNode. ");
	I_Method0(osg::Vec3, getCameraOffset,
	          Properties::NON_VIRTUAL,
	          __osg_Vec3__getCameraOffset,
	          "",
	          "");
	I_Method3(void, setCameraOrientation, IN, float, pitch, IN, float, roll, IN, float, yaw,
	          Properties::NON_VIRTUAL,
	          __void__setCameraOrientation__float__float__float,
	          "",
	          "Set's the (local) orientation of the camera. The default is looking along the +Y axis with +Z up and +X to the right. ");
	I_Method0(osg::Vec3, getCameraOrientation,
	          Properties::NON_VIRTUAL,
	          __osg_Vec3__getCameraOrientation,
	          "",
	          "");
	I_Method4(void, setCameraOrientationQuat, IN, float, x, IN, float, y, IN, float, z, IN, float, w,
	          Properties::NON_VIRTUAL,
	          __void__setCameraOrientationQuat__float__float__float__float,
	          "",
	          "Provides a mechanism to set the orientation with a quaternion ");
	I_Method0(osg::Quat, getCameraOrientationQuat,
	          Properties::NON_VIRTUAL,
	          __osg_Quat__getCameraOrientationQuat,
	          "",
	          "");
	I_Method0(void, ping,
	          Properties::NON_VIRTUAL,
	          __void__ping,
	          "",
	          "Pings the server. ");
	I_Method0(osg::Timer_t, getLastPing,
	          Properties::NON_VIRTUAL,
	          __osg_Timer_t__getLastPing,
	          "",
	          "Returns the timestamp of the last ping in unsigned long long format. ");
	I_Method0(std::vector< lo_message >, getState,
	          Properties::VIRTUAL,
	          __std_vectorT1_lo_message___getState,
	          "",
	          "For each subclass of ReferencedNode, we override the getState() method to fill the vector with the correct set of methods for this particular node ");
	I_SimpleProperty(osg::PositionAttitudeTransform *, CameraAttachmentNode, 
	                 __osg_PositionAttitudeTransform_P1__getCameraAttachmentNode, 
	                 0);
	I_SimpleProperty(osg::Vec3, CameraOffset, 
	                 __osg_Vec3__getCameraOffset, 
	                 0);
	I_SimpleProperty(osg::PositionAttitudeTransform *, CameraOffsetNode, 
	                 __osg_PositionAttitudeTransform_P1__getCameraOffsetNode, 
	                 0);
	I_SimpleProperty(osg::Vec3, CameraOrientation, 
	                 __osg_Vec3__getCameraOrientation, 
	                 0);
	I_SimpleProperty(osg::Quat, CameraOrientationQuat, 
	                 __osg_Quat__getCameraOrientationQuat, 
	                 0);
	I_SimpleProperty(const char *, Description, 
	                 __C5_char_P1__getDescription, 
	                 __void__setDescription__C5_char_P1);
	I_SimpleProperty(osg::Timer_t, LastPing, 
	                 __osg_Timer_t__getLastPing, 
	                 0);
	I_SimpleProperty(std::vector< lo_message >, State, 
	                 __std_vectorT1_lo_message___getState, 
	                 0);
	I_PublicMemberProperty(bool, nodepathUpdate);
END_REFLECTOR

