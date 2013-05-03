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

#include <groupnode.h>
#include <scenemanager.h>
#include <spinutil.h>

// Must undefine IN and OUT macros defined in Windows headers
#ifdef IN
#undef IN
#endif
#ifdef OUT
#undef OUT
#endif

BEGIN_VALUE_REFLECTOR(spin::DraggerCallback)
	I_DeclaringFile("groupnode.h");
	I_Constructor1(IN, spin::GroupNode *, g,
	               Properties::NON_EXPLICIT,
	               ____DraggerCallback__GroupNode_P1,
	               "",
	               "");
	I_Method1(bool, receive, IN, const osgManipulator::MotionCommand &, command,
	          Properties::NON_VIRTUAL,
	          __bool__receive__C5_osgManipulator_MotionCommand_R1,
	          "",
	          "");
END_REFLECTOR

BEGIN_ENUM_REFLECTOR(spin::GroupNode::InteractionMode)
	I_DeclaringFile("groupnode.h");
	I_EnumLabel(spin::GroupNode::STATIC);
	I_EnumLabel(spin::GroupNode::PASSTHRU);
	I_EnumLabel(spin::GroupNode::SELECT);
	I_EnumLabel(spin::GroupNode::DRAG);
	I_EnumLabel(spin::GroupNode::THROW);
	I_EnumLabel(spin::GroupNode::DRAW);
END_REFLECTOR

BEGIN_ENUM_REFLECTOR(spin::GroupNode::globalsReportMode)
	I_DeclaringFile("groupnode.h");
	I_EnumLabel(spin::GroupNode::NONE);
	I_EnumLabel(spin::GroupNode::GLOBAL_6DOF);
	I_EnumLabel(spin::GroupNode::GLOBAL_ALL);
END_REFLECTOR

BEGIN_ENUM_REFLECTOR(spin::GroupNode::velocityMode)
	I_DeclaringFile("groupnode.h");
	I_EnumLabel(spin::GroupNode::TRANSLATE);
	I_EnumLabel(spin::GroupNode::MOVE);
END_REFLECTOR

BEGIN_ENUM_REFLECTOR(spin::GroupNode::ComputationMode)
	I_DeclaringFile("groupnode.h");
	I_EnumLabel(spin::GroupNode::SERVER_SIDE);
	I_EnumLabel(spin::GroupNode::CLIENT_SIDE);
END_REFLECTOR

BEGIN_ENUM_REFLECTOR(spin::GroupNode::OrientationMode)
	I_DeclaringFile("groupnode.h");
	I_EnumLabel(spin::GroupNode::NORMAL);
	I_EnumLabel(spin::GroupNode::POINT_TO_TARGET);
	I_EnumLabel(spin::GroupNode::POINT_TO_TARGET_CENTROID);
	I_EnumLabel(spin::GroupNode::POINT_TO_ORIGIN);
END_REFLECTOR

BEGIN_OBJECT_REFLECTOR(spin::GroupNode)
	I_DeclaringFile("groupnode.h");
	I_BaseType(spin::ReferencedNode);
	I_Constructor2(IN, spin::SceneManager *, sceneManager, IN, const char *, initID,
	               ____GroupNode__SceneManager_P1__C5_char_P1,
	               "",
	               " param initID will be converted into a t_symbol  ");
	I_Method1(void, callbackUpdate, IN, osg::NodeVisitor *, nv,
	          Properties::VIRTUAL,
	          __void__callbackUpdate__osg_NodeVisitor_P1,
	          "",
	          "For nodes that require regular programmatic control, there is a callback that is evaluated with every refresh. This function can thus be used for animations, or any other periodic updates.Note that changes to the scene graph structure (eg, moving/deleting nodes should NOT be done within this callback because traversals stacks will become corrupted. The technique is rather to enable a flag and then do the actual change in the SceneManager::updateGraph() method. ");
	I_Method1(void, setUpdateRate, IN, float, seconds,
	          Properties::VIRTUAL,
	          __void__setUpdateRate__float,
	          "",
	          "The update rate tells the server how fast to send both velocity updates and reporting. Generally, we want to throttle network messages on the server-side so that messages don't flood the network when there is a lot of activity. ");
	I_Method0(float, getUpdateRate,
	          Properties::NON_VIRTUAL,
	          __float__getUpdateRate,
	          "",
	          "Returns the currently-set update rate. ");
	I_MethodWithDefaults1(void, updateNodePath, IN, bool, updateChildren, true,
	                      Properties::VIRTUAL,
	                      __void__updateNodePath__bool,
	                      "",
	                      "IMPORTANT: subclasses of ReferencedNode are allowed to contain complicated subgraphs and can also change their attachmentNode so that children are attached anywhere in that subgraph. If that is the case, the updateNodePath() function MUST be overridden, and extra nodes must be manually pushed onto currentNodePath_. ");
	I_Method5(void, mouseEvent, IN, int, event, IN, int, keyMask, IN, int, buttonMask, IN, float, x, IN, float, y,
	          Properties::NON_VIRTUAL,
	          __void__mouseEvent__int__int__int__float__float,
	          "",
	          "");
	I_Method7(void, event, IN, int, event, IN, const char *, userString, IN, float, eData1, IN, float, eData2, IN, float, x, IN, float, y, IN, float, z,
	          Properties::NON_VIRTUAL,
	          __void__event__int__C5_char_P1__float__float__float__float__float,
	          "",
	          "");
	I_Method2(void, setLock, IN, const char *, userString, IN, int, lock,
	          Properties::NON_VIRTUAL,
	          __void__setLock__C5_char_P1__int,
	          "",
	          "");
	I_Method0(void, debug,
	          Properties::VIRTUAL,
	          __void__debug,
	          "",
	          "Print debug information about the node to standard out (when running in console mode). It may be possible to redirect this to a text box for GUI logs. ");
	I_Method1(void, setStateSetFromFile, IN, const char *, filename,
	          Properties::NON_VIRTUAL,
	          __void__setStateSetFromFile__C5_char_P1,
	          "",
	          "setStateSetFromFile guesses the type of stateset from the filename extension, creates a new stateset of that type and assigns it to this node ");
	I_Method1(void, setStateSet, IN, const char *, s,
	          Properties::NON_VIRTUAL,
	          __void__setStateSet__C5_char_P1,
	          "",
	          "Assign an existing stateset to this node ");
	I_Method0(spin::t_symbol *, getStateSetSymbol,
	          Properties::NON_VIRTUAL,
	          __t_symbol_P1__getStateSetSymbol,
	          "",
	          "");
	I_Method0(void, updateStateSet,
	          Properties::VIRTUAL,
	          __void__updateStateSet,
	          "",
	          "This method actually applies the stateset to the subgraph, replacing any existing stateset with this one. The setStateSet and setStateSetFromFile methods just set the stateset_ symbol, while updateStateSet does the actual work.Override this method in subclasses in order to change how stateset should be applied. For example, to which node in the subgraph it should be attached, or whether it should be merged with the existing stateset (rather than merged).By default it is applied to the mainTransform_. ");
	I_Method1(void, setReportMode, IN, spin::GroupNode::globalsReportMode, mode,
	          Properties::NON_VIRTUAL,
	          __void__setReportMode__globalsReportMode,
	          "",
	          "Sets the report mode with reference to the globalsReportMode enum. ");
	I_Method1(void, setInteractionMode, IN, spin::GroupNode::InteractionMode, mode,
	          Properties::NON_VIRTUAL,
	          __void__setInteractionMode__InteractionMode,
	          "",
	          "Sets the interaction mode with reference to the InteractionMode enum. ");
	I_Method1(void, setComputationMode, IN, spin::GroupNode::ComputationMode, mode,
	          Properties::NON_VIRTUAL,
	          __void__setComputationMode__ComputationMode,
	          "",
	          "Sets the Computation mode as either server or client side with respect to the ComputationMode enum. ");
	I_Method0(int, getComputationMode,
	          Properties::NON_VIRTUAL,
	          __int__getComputationMode,
	          "",
	          "Returns the currently-set computation mode with respect to the ComputationMode enum. ");
	I_Method3(void, setClipping, IN, float, x, IN, float, y, IN, float, z,
	          Properties::NON_VIRTUAL,
	          __void__setClipping__float__float__float,
	          "",
	          "Set a clipping rectangle for the model so that geometry outside of the region (+-x, +-y, +-z) will not be shown (or used in interactive events) ");
	I_Method3(void, setTranslation, IN, float, x, IN, float, y, IN, float, z,
	          Properties::VIRTUAL,
	          __void__setTranslation__float__float__float,
	          "",
	          "The local translation offset for this node with respect to it's parent ");
	I_Method1(void, setOrientationMode, IN, spin::GroupNode::OrientationMode, m,
	          Properties::NON_VIRTUAL,
	          __void__setOrientationMode__OrientationMode,
	          "",
	          "Set the OrientationMode of the node, which will be applied after every transformation. ");
	I_Method0(int, getOrientationMode,
	          Properties::NON_VIRTUAL,
	          __int__getOrientationMode,
	          "",
	          "Returns the currently-set Orientation Mode, with respect to the OrientationMode enum. ");
	I_Method1(void, setOrientationTarget, IN, const char *, target,
	          Properties::NON_VIRTUAL,
	          __void__setOrientationTarget__C5_char_P1,
	          "",
	          "");
	I_Method0(char *, getOrientationTarget,
	          Properties::NON_VIRTUAL,
	          __char_P1__getOrientationTarget,
	          "",
	          "");
	I_Method3(void, setOrientation, IN, float, pitch, IN, float, roll, IN, float, yaw,
	          Properties::VIRTUAL,
	          __void__setOrientation__float__float__float,
	          "",
	          "The local orientation offset for this node with respect to its parent ");
	I_Method4(void, setOrientationQuat, IN, float, x, IN, float, y, IN, float, z, IN, float, w,
	          Properties::VIRTUAL,
	          __void__setOrientationQuat__float__float__float__float,
	          "",
	          "Set the orientation offset as a quaternion ");
	I_Method3(void, setScale, IN, float, x, IN, float, y, IN, float, z,
	          Properties::VIRTUAL,
	          __void__setScale__float__float__float,
	          "",
	          "A grouped scale operation. ");
	I_Method3(void, setVelocity, IN, float, dx, IN, float, dy, IN, float, dz,
	          Properties::VIRTUAL,
	          __void__setVelocity__float__float__float,
	          "",
	          "A translational velocity (m/s). This is computed in the callbackUpdate() function. ");
	I_Method1(void, setVelocityMode, IN, spin::GroupNode::velocityMode, mode,
	          Properties::VIRTUAL,
	          __void__setVelocityMode__velocityMode,
	          "",
	          "Applying velocity to an object can either result in translational motion, where velocityMode is TRANSLATE (0). This is the default and applies and motion is relative to the local coordinate system of the node. When velocityMode is MOVE (1), then motion is relative to the current orientation of the node, analogous to the move() command. ");
	I_Method3(void, setSpin, IN, float, dp, IN, float, dr, IN, float, dy,
	          Properties::VIRTUAL,
	          __void__setSpin__float__float__float,
	          "",
	          "A rotational velocity (deg/sec), computed in callbackUpdate(). ");
	I_Method1(void, setDamping, IN, float, d,
	          Properties::VIRTUAL,
	          __void__setDamping__float,
	          "",
	          "A Damping value (negative acceleration) that gets applied to velocity and spin over time. Units are in -m/sec2 or -deg/sec2, meaning that: zero damping will not change velocity/spin valuesany positive value will decrease the speeds  ");
	I_Method3(void, translate, IN, float, x, IN, float, y, IN, float, z,
	          Properties::VIRTUAL,
	          __void__translate__float__float__float,
	          "",
	          "The translate command increments the node's current translation values (ie, it's position in the scene with respect to it's parent) ");
	I_Method3(void, move, IN, float, x, IN, float, y, IN, float, z,
	          Properties::VIRTUAL,
	          __void__move__float__float__float,
	          "",
	          "The move command adds a relative translation with respect to the node's current orientation. That is, the node will translate along it's direction vector by the supplied number of units. ");
	I_Method3(void, rotate, IN, float, dPitch, IN, float, dRoll, IN, float, dYaw,
	          Properties::VIRTUAL,
	          __void__rotate__float__float__float,
	          "",
	          "The rotate command adds to the (absolute) orientation of the node ");
	I_Method3(void, addRotation, IN, float, dPitch, IN, float, dRoll, IN, float, dYaw,
	          Properties::VIRTUAL,
	          __void__addRotation__float__float__float,
	          "",
	          "The addRotation command adds a (relative) rotation to the node's current orientation. ");
	I_MethodWithDefaults5(void, translateTo, IN, float, x, , IN, float, y, , IN, float, z, , IN, float, time, , IN, const char *, motion, "Linear",
	                      Properties::VIRTUAL,
	                      __void__translateTo__float__float__float__float__C5_char_P1,
	                      "",
	                      "Instead of instantaneous setTranslation, this method uses an ease motion to animate the node to the target position. ");
	I_Method1(void, setManipulator, IN, const char *, manipulatorType,
	          Properties::VIRTUAL,
	          __void__setManipulator__C5_char_P1,
	          "",
	          "");
	I_Method0(const char *, getManipulator,
	          Properties::NON_VIRTUAL,
	          __C5_char_P1__getManipulator,
	          "",
	          "");
	I_Method16(void, setManipulatorMatrix, IN, float, a00, IN, float, a01, IN, float, a02, IN, float, a03, IN, float, a10, IN, float, a11, IN, float, a12, IN, float, a13, IN, float, a20, IN, float, a21, IN, float, a22, IN, float, a23, IN, float, a30, IN, float, a31, IN, float, a32, IN, float, a33,
	           Properties::VIRTUAL,
	           __void__setManipulatorMatrix__float__float__float__float__float__float__float__float__float__float__float__float__float__float__float__float,
	           "",
	           "");
	I_Method1(void, setBroadcastLock, IN, bool, lock,
	          Properties::NON_VIRTUAL,
	          __void__setBroadcastLock__bool,
	          "",
	          "");
	I_Method0(int, getReportMode,
	          Properties::NON_VIRTUAL,
	          __int__getReportMode,
	          "",
	          "Returns the currently-set Report Mode with reference to the globalsReportMode enum. ");
	I_Method0(int, getInteractionMode,
	          Properties::NON_VIRTUAL,
	          __int__getInteractionMode,
	          "",
	          "Returns the currently-set Interaction Mode with reference to the InteractionMode enum. ");
	I_Method0(osg::Vec3, getClipping,
	          Properties::NON_VIRTUAL,
	          __osg_Vec3__getClipping,
	          "",
	          "");
	I_Method0(osg::Vec3, getOrientation,
	          Properties::NON_VIRTUAL,
	          __osg_Vec3__getOrientation,
	          "",
	          "Returns the currently-set local orientation offset for this node with respect to its parent. ");
	I_Method0(osg::Vec3, getTranslation,
	          Properties::VIRTUAL,
	          __osg_Vec3__getTranslation,
	          "",
	          "Returns the currently-set local translation offset for this node with respect to its parent. ");
	I_Method0(osg::Quat, getOrientationQuat,
	          Properties::VIRTUAL,
	          __osg_Quat__getOrientationQuat,
	          "",
	          "Returns the currently-set ");
	I_Method0(osg::Vec3, getScale,
	          Properties::VIRTUAL,
	          __osg_Vec3__getScale,
	          "",
	          "");
	I_Method0(osg::Vec3, getVelocity,
	          Properties::NON_VIRTUAL,
	          __osg_Vec3__getVelocity,
	          "",
	          "");
	I_Method0(int, getVelocityMode,
	          Properties::NON_VIRTUAL,
	          __int__getVelocityMode,
	          "",
	          "");
	I_Method0(float, getDamping,
	          Properties::NON_VIRTUAL,
	          __float__getDamping,
	          "",
	          "");
	I_Method0(osg::Matrix, getGlobalMatrix,
	          Properties::NON_VIRTUAL,
	          __osg_Matrix__getGlobalMatrix,
	          "",
	          "");
	I_Method0(osg::Vec3, getCenter,
	          Properties::NON_VIRTUAL,
	          __osg_Vec3__getCenter,
	          "",
	          "");
	I_MethodWithDefaults1(bool, dumpGlobals, IN, bool, forced, false,
	                      Properties::VIRTUAL,
	                      __bool__dumpGlobals__bool,
	                      "",
	                      "The dumpGlobals method results in a broadcast of this node's translation and orientation. It is called by callbackUpdate() every frame, however the 'forced' flag will be set to false, so it will only send a message if the node's matrix has changed. If the 'forced' flag is set to true, it will definitely result in a message broadcast. This should only be used when necessary (eg, when a stateDump is requested).Note: the return value is only to fool wx so that it doesn't consider this as an editable property. ");
	I_Method0(std::vector< lo_message >, getState,
	          Properties::VIRTUAL,
	          __std_vectorT1_lo_message___getState,
	          "",
	          "For each subclass of ReferencedNode, we override the getState() method to fill the vector with the correct set of methods for this particular node ");
	I_Method0(void, stateDump,
	          Properties::VIRTUAL,
	          __void__stateDump,
	          "",
	          "We override stateDump so that we can additionally force a dumpGlobals() call whenever a dump is requested ");
	I_Method0(osg::MatrixTransform *, getTransform,
	          Properties::NON_VIRTUAL,
	          __osg_MatrixTransform_P1__getTransform,
	          "",
	          "");
	I_Method0(void, updateDraggerMatrix,
	          Properties::NON_VIRTUAL,
	          __void__updateDraggerMatrix,
	          "",
	          "");
	I_Method1(void, removeOrientationTargetter, IN, spin::GroupNode *, gn,
	          Properties::NON_VIRTUAL,
	          __void__removeOrientationTargetter__GroupNode_P1,
	          "",
	          "");
	I_Method1(void, addOrientationTargetter, IN, spin::GroupNode *, gn,
	          Properties::NON_VIRTUAL,
	          __void__addOrientationTargetter__GroupNode_P1,
	          "",
	          "");
	I_Method0(void, applyOrientationModeToTargetters,
	          Properties::NON_VIRTUAL,
	          __void__applyOrientationModeToTargetters,
	          "",
	          "");
	I_ProtectedMethod0(void, updateQuat,
	                   Properties::NON_VIRTUAL,
	                   Properties::NON_CONST,
	                   __void__updateQuat,
	                   "",
	                   "");
	I_ProtectedMethod0(void, updateMatrix,
	                   Properties::NON_VIRTUAL,
	                   Properties::NON_CONST,
	                   __void__updateMatrix,
	                   "",
	                   "");
	I_ProtectedMethod0(void, drawManipulator,
	                   Properties::NON_VIRTUAL,
	                   Properties::NON_CONST,
	                   __void__drawManipulator,
	                   "",
	                   "");
	I_ProtectedMethod0(void, applyOrientationMode,
	                   Properties::NON_VIRTUAL,
	                   Properties::NON_CONST,
	                   __void__applyOrientationMode,
	                   "",
	                   "");
	I_SimpleProperty(bool, BroadcastLock, 
	                 0, 
	                 __void__setBroadcastLock__bool);
	I_SimpleProperty(osg::Vec3, Center, 
	                 __osg_Vec3__getCenter, 
	                 0);
	I_SimpleProperty(osg::Vec3, Clipping, 
	                 __osg_Vec3__getClipping, 
	                 0);
	I_SimpleProperty(int, ComputationMode, 
	                 __int__getComputationMode, 
	                 0);
	I_SimpleProperty(float, Damping, 
	                 __float__getDamping, 
	                 __void__setDamping__float);
	I_SimpleProperty(osg::Matrix, GlobalMatrix, 
	                 __osg_Matrix__getGlobalMatrix, 
	                 0);
	I_SimpleProperty(int, InteractionMode, 
	                 __int__getInteractionMode, 
	                 0);
	I_SimpleProperty(const char *, Manipulator, 
	                 __C5_char_P1__getManipulator, 
	                 __void__setManipulator__C5_char_P1);
	I_SimpleProperty(osg::Vec3, Orientation, 
	                 __osg_Vec3__getOrientation, 
	                 0);
	I_SimpleProperty(int, OrientationMode, 
	                 __int__getOrientationMode, 
	                 0);
	I_SimpleProperty(osg::Quat, OrientationQuat, 
	                 __osg_Quat__getOrientationQuat, 
	                 0);
	I_SimpleProperty(char *, OrientationTarget, 
	                 __char_P1__getOrientationTarget, 
	                 0);
	I_SimpleProperty(int, ReportMode, 
	                 __int__getReportMode, 
	                 0);
	I_SimpleProperty(osg::Vec3, Scale, 
	                 __osg_Vec3__getScale, 
	                 0);
	I_SimpleProperty(std::vector< lo_message >, State, 
	                 __std_vectorT1_lo_message___getState, 
	                 0);
	I_SimpleProperty(const char *, StateSet, 
	                 0, 
	                 __void__setStateSet__C5_char_P1);
	I_SimpleProperty(const char *, StateSetFromFile, 
	                 0, 
	                 __void__setStateSetFromFile__C5_char_P1);
	I_SimpleProperty(spin::t_symbol *, StateSetSymbol, 
	                 __t_symbol_P1__getStateSetSymbol, 
	                 0);
	I_SimpleProperty(osg::MatrixTransform *, Transform, 
	                 __osg_MatrixTransform_P1__getTransform, 
	                 0);
	I_SimpleProperty(osg::Vec3, Translation, 
	                 __osg_Vec3__getTranslation, 
	                 0);
	I_SimpleProperty(float, UpdateRate, 
	                 __float__getUpdateRate, 
	                 __void__setUpdateRate__float);
	I_SimpleProperty(osg::Vec3, Velocity, 
	                 __osg_Vec3__getVelocity, 
	                 0);
	I_SimpleProperty(int, VelocityMode, 
	                 __int__getVelocityMode, 
	                 0);
END_REFLECTOR
