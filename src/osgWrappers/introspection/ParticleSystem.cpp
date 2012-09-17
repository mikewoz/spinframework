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

#include <ParticleSystem.h>
#include <SceneManager.h>

// Must undefine IN and OUT macros defined in Windows headers
#ifdef IN
#undef IN
#endif
#ifdef OUT
#undef OUT
#endif

BEGIN_ENUM_REFLECTOR(spin::ParticleSystem::PlacerType)
	I_DeclaringFile("ParticleSystem.h");
	I_EnumLabel(spin::ParticleSystem::RADIAL);
	I_EnumLabel(spin::ParticleSystem::CUBIC);
	I_EnumLabel(spin::ParticleSystem::LINEAR);
END_REFLECTOR

BEGIN_OBJECT_REFLECTOR(spin::ParticleSystem)
	I_DeclaringFile("ParticleSystem.h");
	I_BaseType(spin::ConstraintsNode);
	I_Constructor2(IN, spin::SceneManager *, sceneManager, IN, const char *, initID,
	               ____ParticleSystem__SceneManager_P1__C5_char_P1,
	               "",
	               "");
	I_Method1(void, callbackUpdate, IN, osg::NodeVisitor *, nv,
	          Properties::VIRTUAL,
	          __void__callbackUpdate__osg_NodeVisitor_P1,
	          "",
	          "For nodes that require regular programmatic control, there is a callback that is evaluated with every refresh. This function can thus be used for animations, or any other periodic updates.Note that changes to the scene graph structure (eg, moving/deleting nodes should NOT be done within this callback because traversals stacks will become corrupted. The technique is rather to enable a flag and then do the actual change in the SceneManager::updateGraph() method. ");
	I_Method1(void, updateNodePath, IN, bool, updateChildren,
	          Properties::VIRTUAL,
	          __void__updateNodePath__bool,
	          "",
	          "IMPORTANT: subclasses of ReferencedNode are allowed to contain complicated subgraphs and can also change their attachmentNode so that children are attached anywhere in that subgraph. If that is the case, the updateNodePath() function MUST be overridden, and extra nodes must be manually pushed onto currentNodePath_. ");
	I_Method0(void, debug,
	          Properties::VIRTUAL,
	          __void__debug,
	          "",
	          "Print debug information about the node to standard out (when running in console mode). It may be possible to redirect this to a text box for GUI logs. ");
	I_Method1(void, setConnected, IN, int, b,
	          Properties::NON_VIRTUAL,
	          __void__setConnected__int,
	          "",
	          "");
	I_Method1(void, setPlacerType, IN, int, type,
	          Properties::NON_VIRTUAL,
	          __void__setPlacerType__int,
	          "",
	          "");
	I_Method0(int, getPlacerType,
	          Properties::NON_VIRTUAL,
	          __int__getPlacerType,
	          "",
	          "");
	I_Method2(void, setRadialRange, IN, float, min, IN, float, max,
	          Properties::NON_VIRTUAL,
	          __void__setRadialRange__float__float,
	          "",
	          "");
	I_Method0(osg::Vec2, getRadialRange,
	          Properties::NON_VIRTUAL,
	          __osg_Vec2__getRadialRange,
	          "",
	          "");
	I_Method2(void, setRadialPhiRange, IN, float, min, IN, float, max,
	          Properties::NON_VIRTUAL,
	          __void__setRadialPhiRange__float__float,
	          "",
	          "");
	I_Method0(osg::Vec2, getRadialPhiRange,
	          Properties::NON_VIRTUAL,
	          __osg_Vec2__getRadialPhiRange,
	          "",
	          "");
	I_Method2(void, setCubicXRange, IN, float, min, IN, float, max,
	          Properties::NON_VIRTUAL,
	          __void__setCubicXRange__float__float,
	          "",
	          "");
	I_Method0(osg::Vec2, getCubicXRange,
	          Properties::NON_VIRTUAL,
	          __osg_Vec2__getCubicXRange,
	          "",
	          "");
	I_Method2(void, setCubicYRange, IN, float, min, IN, float, max,
	          Properties::NON_VIRTUAL,
	          __void__setCubicYRange__float__float,
	          "",
	          "");
	I_Method0(osg::Vec2, getCubicYRange,
	          Properties::NON_VIRTUAL,
	          __osg_Vec2__getCubicYRange,
	          "",
	          "");
	I_Method2(void, setCubicZRange, IN, float, min, IN, float, max,
	          Properties::NON_VIRTUAL,
	          __void__setCubicZRange__float__float,
	          "",
	          "");
	I_Method0(osg::Vec2, getCubicZRange,
	          Properties::NON_VIRTUAL,
	          __osg_Vec2__getCubicZRange,
	          "",
	          "");
	I_Method3(void, addLinearPlacerVertex, IN, float, x, IN, float, y, IN, float, z,
	          Properties::NON_VIRTUAL,
	          __void__addLinearPlacerVertex__float__float__float,
	          "",
	          "");
	I_Method0(void, removeLinearPlacerVertices,
	          Properties::NON_VIRTUAL,
	          __void__removeLinearPlacerVertices,
	          "",
	          "");
	I_Method1(void, enableOrbiter, IN, int, b,
	          Properties::NON_VIRTUAL,
	          __void__enableOrbiter__int,
	          "",
	          "The orbiter forces particles in the orbit around a point. ");
	I_Method0(int, getEnabledOrbiter,
	          Properties::NON_VIRTUAL,
	          __int__getEnabledOrbiter,
	          "",
	          "");
	I_Method1(void, enableAttractor, IN, int, b,
	          Properties::NON_VIRTUAL,
	          __void__enableAttractor__int,
	          "",
	          "Applies an attractive force towards a specific point. ");
	I_Method0(int, getEnabledAttractor,
	          Properties::NON_VIRTUAL,
	          __int__getEnabledAttractor,
	          "",
	          "");
	I_Method1(void, enableOscillator, IN, int, b,
	          Properties::NON_VIRTUAL,
	          __void__enableOscillator__int,
	          "",
	          "Applies an oscilating motion. ");
	I_Method0(int, getEnabledOscillator,
	          Properties::NON_VIRTUAL,
	          __int__getEnabledOscillator,
	          "",
	          "");
	I_Method1(void, enableAccelerator, IN, int, b,
	          Properties::NON_VIRTUAL,
	          __void__enableAccelerator__int,
	          "",
	          "Applies a constant acceleration to the particles (eg, gravity). ");
	I_Method0(int, getEnabledAccelerator,
	          Properties::NON_VIRTUAL,
	          __int__getEnabledAccelerator,
	          "",
	          "");
	I_Method1(void, enableAngularAccelerator, IN, int, b,
	          Properties::NON_VIRTUAL,
	          __void__enableAngularAccelerator__int,
	          "",
	          "Applies a constant angular acceleration to the particles. ");
	I_Method0(int, getEnabledAngularAccelerator,
	          Properties::NON_VIRTUAL,
	          __int__getEnabledAngularAccelerator,
	          "",
	          "");
	I_Method1(void, enableAngularDamping, IN, int, b,
	          Properties::NON_VIRTUAL,
	          __void__enableAngularDamping__int,
	          "",
	          "Applies damping constant to particle's angular velocity. ");
	I_Method0(int, getEnabledAngularDamping,
	          Properties::NON_VIRTUAL,
	          __int__getEnabledAngularDamping,
	          "",
	          "");
	I_Method1(void, enableDamping, IN, int, b,
	          Properties::NON_VIRTUAL,
	          __void__enableDamping__int,
	          "",
	          "Applies damping constant to particle's velocity. ");
	I_Method0(int, getEnabledDamping,
	          Properties::NON_VIRTUAL,
	          __int__getEnabledDamping,
	          "",
	          "");
	I_Method1(void, enableFluidFriction, IN, int, b,
	          Properties::NON_VIRTUAL,
	          __void__enableFluidFriction__int,
	          "",
	          "Simulates the friction of a fluid.By using this operator you can let the particles move in a fluid of a given density and viscosity. There are two functions to quickly setup the parameters for pure water and air. You can decide whether to compute the forces using the particle's physical radius or another value, by calling the setOverrideRadius() method. ");
	I_Method0(int, getEnabledFluidFriction,
	          Properties::NON_VIRTUAL,
	          __int__getEnabledFluidFriction,
	          "",
	          "");
	I_Method1(void, enableExplosion, IN, int, b,
	          Properties::NON_VIRTUAL,
	          __void__enableExplosion__int,
	          "",
	          "Exerts force on each particle away from the explosion center. ");
	I_Method0(int, getEnabledExplosion,
	          Properties::NON_VIRTUAL,
	          __int__getEnabledExplosion,
	          "",
	          "");
	I_Method1(void, enableForce, IN, int, b,
	          Properties::NON_VIRTUAL,
	          __void__enableForce__int,
	          "",
	          "Applies a constant force to the particles. Remember that if the mass of particles is expressed in kg and the lengths are expressed in meters, then the force should be expressed in Newtons. ");
	I_Method0(int, getEnabledForce,
	          Properties::NON_VIRTUAL,
	          __int__getEnabledForce,
	          "",
	          "");
	I_Method1(void, enableBouncer, IN, int, b,
	          Properties::NON_VIRTUAL,
	          __void__enableBouncer__int,
	          "",
	          "Can affect the particle's velocity to make it rebound. ");
	I_Method0(int, getEnabledBouncer,
	          Properties::NON_VIRTUAL,
	          __int__getEnabledBouncer,
	          "",
	          "");
	I_Method3(void, setAccel, IN, float, x, IN, float, y, IN, float, z,
	          Properties::NON_VIRTUAL,
	          __void__setAccel__float__float__float,
	          "",
	          "");
	I_Method3(void, setAngularAccel, IN, float, x, IN, float, y, IN, float, z,
	          Properties::NON_VIRTUAL,
	          __void__setAngularAccel__float__float__float,
	          "",
	          "");
	I_Method1(void, setAngularDamping, IN, float, d,
	          Properties::NON_VIRTUAL,
	          __void__setAngularDamping__float,
	          "",
	          "");
	I_Method1(void, setDamping, IN, float, d,
	          Properties::VIRTUAL,
	          __void__setDamping__float,
	          "",
	          "A Damping value (negative acceleration) that gets applied to velocity and spin over time. Units are in -m/sec2 or -deg/sec2, meaning that: zero damping will not change velocity/spin valuesany positive value will decrease the speeds  ");
	I_Method3(void, setOrbitCenter, IN, float, x, IN, float, y, IN, float, z,
	          Properties::NON_VIRTUAL,
	          __void__setOrbitCenter__float__float__float,
	          "",
	          "");
	I_Method1(void, setOrbitMagnitude, IN, float, mag,
	          Properties::NON_VIRTUAL,
	          __void__setOrbitMagnitude__float,
	          "",
	          "");
	I_Method1(void, setOrbitEpsilon, IN, float, eps,
	          Properties::NON_VIRTUAL,
	          __void__setOrbitEpsilon__float,
	          "",
	          "");
	I_Method1(void, setOrbitMaxRadius, IN, float, max,
	          Properties::NON_VIRTUAL,
	          __void__setOrbitMaxRadius__float,
	          "",
	          "");
	I_Method3(void, setAttractorCenter, IN, float, x, IN, float, y, IN, float, z,
	          Properties::NON_VIRTUAL,
	          __void__setAttractorCenter__float__float__float,
	          "",
	          "");
	I_Method1(void, setAttractorMagnitude, IN, float, mag,
	          Properties::NON_VIRTUAL,
	          __void__setAttractorMagnitude__float,
	          "",
	          "");
	I_Method1(void, setAttractorRatio, IN, float, ratio,
	          Properties::NON_VIRTUAL,
	          __void__setAttractorRatio__float,
	          "",
	          "");
	I_Method1(void, setAttractorKillSink, IN, int, kill,
	          Properties::NON_VIRTUAL,
	          __void__setAttractorKillSink__int,
	          "",
	          "");
	I_Method1(void, setOscillatorAmplitude, IN, float, amp,
	          Properties::NON_VIRTUAL,
	          __void__setOscillatorAmplitude__float,
	          "",
	          "");
	I_Method1(void, setOscillatorFrequency, IN, float, f,
	          Properties::NON_VIRTUAL,
	          __void__setOscillatorFrequency__float,
	          "",
	          "");
	I_Method1(void, setOscillatorLockAngle, IN, int, lock,
	          Properties::NON_VIRTUAL,
	          __void__setOscillatorLockAngle__int,
	          "",
	          "");
	I_Method1(void, setExplosionTarget, IN, const char *, targetID,
	          Properties::NON_VIRTUAL,
	          __void__setExplosionTarget__C5_char_P1,
	          "",
	          "");
	I_Method0(std::string, getExplosionTarget,
	          Properties::NON_VIRTUAL,
	          __std_string__getExplosionTarget,
	          "",
	          "");
	I_Method1(void, setExplosionDebugView, IN, int, b,
	          Properties::NON_VIRTUAL,
	          __void__setExplosionDebugView__int,
	          "",
	          "");
	I_Method0(int, getExplosionDebugView,
	          Properties::NON_VIRTUAL,
	          __int__getExplosionDebugView,
	          "",
	          "");
	I_Method3(void, setExplosionCenter, IN, float, x, IN, float, y, IN, float, z,
	          Properties::NON_VIRTUAL,
	          __void__setExplosionCenter__float__float__float,
	          "sets the center of the explosion (Note: an explosion target overrides this) ",
	          "");
	I_Method1(void, setExplosionRadius, IN, float, r,
	          Properties::NON_VIRTUAL,
	          __void__setExplosionRadius__float,
	          "radius defines how far away the shockwave peaks ",
	          "");
	I_Method1(void, setExplosionMagnitude, IN, float, mag,
	          Properties::NON_VIRTUAL,
	          __void__setExplosionMagnitude__float,
	          "magnitude defines the amount of force exerted on the particles. Try 100+ ",
	          "");
	I_Method1(void, setExplosionEpsilon, IN, float, eps,
	          Properties::NON_VIRTUAL,
	          __void__setExplosionEpsilon__float,
	          "epsilon sets the distance from the center ",
	          "");
	I_Method1(void, setExplosionSigma, IN, float, s,
	          Properties::NON_VIRTUAL,
	          __void__setExplosionSigma__float,
	          "sigma (in degrees) determines the broadness of the explosion shockwave ",
	          "");
	I_Method3(void, setForce, IN, float, x, IN, float, y, IN, float, z,
	          Properties::NON_VIRTUAL,
	          __void__setForce__float__float__float,
	          "",
	          "");
	I_Method1(void, setFluidDensity, IN, float, d,
	          Properties::NON_VIRTUAL,
	          __void__setFluidDensity__float,
	          "",
	          "");
	I_Method1(void, setFluidViscosity, IN, float, v,
	          Properties::NON_VIRTUAL,
	          __void__setFluidViscosity__float,
	          "",
	          "");
	I_Method3(void, setFluidDirection, IN, float, x, IN, float, y, IN, float, z,
	          Properties::NON_VIRTUAL,
	          __void__setFluidDirection__float__float__float,
	          "",
	          "");
	I_Method1(void, addBounceTarget, IN, const char *, nodeID,
	          Properties::NON_VIRTUAL,
	          __void__addBounceTarget__C5_char_P1,
	          "",
	          "");
	I_Method1(void, removeBounceTarget, IN, const char *, nodeID,
	          Properties::NON_VIRTUAL,
	          __void__removeBounceTarget__C5_char_P1,
	          "",
	          "");
	I_Method0(void, removeAllBounceTargets,
	          Properties::NON_VIRTUAL,
	          __void__removeAllBounceTargets,
	          "",
	          "");
	I_Method6(void, addBouncePlane, IN, float, normalX, IN, float, normalY, IN, float, normalZ, IN, float, x, IN, float, y, IN, float, z,
	          Properties::NON_VIRTUAL,
	          __void__addBouncePlane__float__float__float__float__float__float,
	          "",
	          "");
	I_Method6(void, addBounceBox, IN, float, minX, IN, float, minY, IN, float, minZ, IN, float, maxX, IN, float, maxY, IN, float, maxZ,
	          Properties::NON_VIRTUAL,
	          __void__addBounceBox__float__float__float__float__float__float,
	          "",
	          "");
	I_Method4(void, addBounceSphere, IN, float, x, IN, float, y, IN, float, z, IN, float, radius,
	          Properties::NON_VIRTUAL,
	          __void__addBounceSphere__float__float__float__float,
	          "",
	          "");
	I_Method0(void, removeAllBouncers,
	          Properties::NON_VIRTUAL,
	          __void__removeAllBouncers,
	          "",
	          "");
	I_Method1(void, setBounceFriction, IN, float, f,
	          Properties::NON_VIRTUAL,
	          __void__setBounceFriction__float,
	          "",
	          "");
	I_Method1(void, setBounceResilience, IN, float, r,
	          Properties::NON_VIRTUAL,
	          __void__setBounceResilience__float,
	          "",
	          "");
	I_Method1(void, setBounceCutoff, IN, float, v,
	          Properties::NON_VIRTUAL,
	          __void__setBounceCutoff__float,
	          "",
	          "");
	I_Method3(void, setTranslation, IN, float, x, IN, float, y, IN, float, z,
	          Properties::VIRTUAL,
	          __void__setTranslation__float__float__float,
	          "",
	          "The local translation offset for this node with respect to it's parent ");
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
	I_Method0(osg::Vec3, getTranslation,
	          Properties::VIRTUAL,
	          __osg_Vec3__getTranslation,
	          "",
	          "Returns the local translation offset with respect to the node's parent. ");
	I_Method0(void, updateStateSet,
	          Properties::VIRTUAL,
	          __void__updateStateSet,
	          "",
	          "This method actually applies the stateset to the subgraph, replacing any existing stateset with this one. The setStateSet and setStateSetFromFile methods just set the stateset_ symbol, while updateStateSet does the actual work.Override this method in subclasses in order to change how stateset should be applied. For example, to which node in the subgraph it should be attached, or whether it should be merged with the existing stateset (rather than merged).By default it is applied to the mainTransform_. ");
	I_Method1(void, setParticleShape, IN, int, shp,
	          Properties::NON_VIRTUAL,
	          __void__setParticleShape__int,
	          "",
	          "");
	I_Method0(int, getParticleShape,
	          Properties::NON_VIRTUAL,
	          __int__getParticleShape,
	          "",
	          "");
	I_Method1(void, setLifeTime, IN, float, seconds,
	          Properties::NON_VIRTUAL,
	          __void__setLifeTime__float,
	          "",
	          "");
	I_Method0(float, getLifeTime,
	          Properties::NON_VIRTUAL,
	          __float__getLifeTime,
	          "",
	          "");
	I_Method1(void, setRadius, IN, float, radius,
	          Properties::NON_VIRTUAL,
	          __void__setRadius__float,
	          "",
	          "");
	I_Method0(float, getRadius,
	          Properties::NON_VIRTUAL,
	          __float__getRadius,
	          "",
	          "");
	I_Method1(void, setMass, IN, float, mass,
	          Properties::NON_VIRTUAL,
	          __void__setMass__float,
	          "",
	          "");
	I_Method0(float, getMass,
	          Properties::NON_VIRTUAL,
	          __float__getMass,
	          "",
	          "");
	I_Method2(void, setParticleSizeRange, IN, float, min, IN, float, max,
	          Properties::NON_VIRTUAL,
	          __void__setParticleSizeRange__float__float,
	          "",
	          "");
	I_Method2(void, setParticleAlphaRange, IN, float, min, IN, float, max,
	          Properties::NON_VIRTUAL,
	          __void__setParticleAlphaRange__float__float,
	          "",
	          "");
	I_Method6(void, setParticleColorRange, IN, float, minR, IN, float, minG, IN, float, minB, IN, float, maxR, IN, float, maxG, IN, float, maxB,
	          Properties::NON_VIRTUAL,
	          __void__setParticleColorRange__float__float__float__float__float__float,
	          "",
	          "");
	I_Method1(void, setEmissive, IN, int, emissiveFlag,
	          Properties::NON_VIRTUAL,
	          __void__setEmissive__int,
	          "",
	          "");
	I_Method0(int, getEmissive,
	          Properties::NON_VIRTUAL,
	          __int__getEmissive,
	          "",
	          "");
	I_Method1(void, setLighting, IN, int, lightingFlag,
	          Properties::NON_VIRTUAL,
	          __void__setLighting__int,
	          "",
	          "");
	I_Method0(int, getLighting,
	          Properties::NON_VIRTUAL,
	          __int__getLighting,
	          "",
	          "");
	I_Method1(void, setUseShaders, IN, int, shaderFlag,
	          Properties::NON_VIRTUAL,
	          __void__setUseShaders__int,
	          "",
	          "");
	I_Method0(int, getUseShaders,
	          Properties::NON_VIRTUAL,
	          __int__getUseShaders,
	          "",
	          "");
	I_Method1(void, setImagePath, IN, const char *, path,
	          Properties::NON_VIRTUAL,
	          __void__setImagePath__C5_char_P1,
	          "",
	          "");
	I_Method0(const char *, getImagePath,
	          Properties::NON_VIRTUAL,
	          __C5_char_P1__getImagePath,
	          "",
	          "");
	I_Method1(void, setImage, IN, const char *, path,
	          Properties::NON_VIRTUAL,
	          __void__setImage__C5_char_P1,
	          "",
	          "");
	I_Method2(void, setFrequencyRange, IN, float, min, IN, float, max,
	          Properties::NON_VIRTUAL,
	          __void__setFrequencyRange__float__float,
	          "",
	          "");
	I_Method0(osg::Vec2, getFrequencyRange,
	          Properties::NON_VIRTUAL,
	          __osg_Vec2__getFrequencyRange,
	          "",
	          "");
	I_Method2(void, setShooterThetaRange, IN, float, min, IN, float, max,
	          Properties::NON_VIRTUAL,
	          __void__setShooterThetaRange__float__float,
	          "",
	          "");
	I_Method0(osg::Vec2, getShooterThetaRange,
	          Properties::NON_VIRTUAL,
	          __osg_Vec2__getShooterThetaRange,
	          "",
	          "");
	I_Method2(void, setShooterPhiRange, IN, float, min, IN, float, max,
	          Properties::NON_VIRTUAL,
	          __void__setShooterPhiRange__float__float,
	          "",
	          "");
	I_Method0(osg::Vec2, getShooterPhiRange,
	          Properties::NON_VIRTUAL,
	          __osg_Vec2__getShooterPhiRange,
	          "",
	          "");
	I_Method2(void, setShooterSpeedRange, IN, float, min, IN, float, max,
	          Properties::NON_VIRTUAL,
	          __void__setShooterSpeedRange__float__float,
	          "",
	          "");
	I_Method0(osg::Vec2, getShooterSpeedRange,
	          Properties::NON_VIRTUAL,
	          __osg_Vec2__getShooterSpeedRange,
	          "",
	          "");
	I_Method6(void, setShooterRotationalSpeedRange, IN, float, minX, IN, float, minY, IN, float, minZ, IN, float, maxX, IN, float, maxY, IN, float, maxZ,
	          Properties::NON_VIRTUAL,
	          __void__setShooterRotationalSpeedRange__float__float__float__float__float__float,
	          "",
	          "");
	I_Method0(std::vector< lo_message >, getState,
	          Properties::VIRTUAL,
	          __std_vectorT1_lo_message___getState,
	          "",
	          "For each subclass of ReferencedNode, we override the getState() method to fill the vector with the correct set of methods for this particular node ");
	I_SimpleProperty(float, AngularDamping, 
	                 0, 
	                 __void__setAngularDamping__float);
	I_SimpleProperty(int, AttractorKillSink, 
	                 0, 
	                 __void__setAttractorKillSink__int);
	I_SimpleProperty(float, AttractorMagnitude, 
	                 0, 
	                 __void__setAttractorMagnitude__float);
	I_SimpleProperty(float, AttractorRatio, 
	                 0, 
	                 __void__setAttractorRatio__float);
	I_SimpleProperty(float, BounceCutoff, 
	                 0, 
	                 __void__setBounceCutoff__float);
	I_SimpleProperty(float, BounceFriction, 
	                 0, 
	                 __void__setBounceFriction__float);
	I_SimpleProperty(float, BounceResilience, 
	                 0, 
	                 __void__setBounceResilience__float);
	I_SimpleProperty(int, Connected, 
	                 0, 
	                 __void__setConnected__int);
	I_SimpleProperty(osg::Vec2, CubicXRange, 
	                 __osg_Vec2__getCubicXRange, 
	                 0);
	I_SimpleProperty(osg::Vec2, CubicYRange, 
	                 __osg_Vec2__getCubicYRange, 
	                 0);
	I_SimpleProperty(osg::Vec2, CubicZRange, 
	                 __osg_Vec2__getCubicZRange, 
	                 0);
	I_SimpleProperty(float, Damping, 
	                 0, 
	                 __void__setDamping__float);
	I_SimpleProperty(int, Emissive, 
	                 __int__getEmissive, 
	                 __void__setEmissive__int);
	I_SimpleProperty(int, EnabledAccelerator, 
	                 __int__getEnabledAccelerator, 
	                 0);
	I_SimpleProperty(int, EnabledAngularAccelerator, 
	                 __int__getEnabledAngularAccelerator, 
	                 0);
	I_SimpleProperty(int, EnabledAngularDamping, 
	                 __int__getEnabledAngularDamping, 
	                 0);
	I_SimpleProperty(int, EnabledAttractor, 
	                 __int__getEnabledAttractor, 
	                 0);
	I_SimpleProperty(int, EnabledBouncer, 
	                 __int__getEnabledBouncer, 
	                 0);
	I_SimpleProperty(int, EnabledDamping, 
	                 __int__getEnabledDamping, 
	                 0);
	I_SimpleProperty(int, EnabledExplosion, 
	                 __int__getEnabledExplosion, 
	                 0);
	I_SimpleProperty(int, EnabledFluidFriction, 
	                 __int__getEnabledFluidFriction, 
	                 0);
	I_SimpleProperty(int, EnabledForce, 
	                 __int__getEnabledForce, 
	                 0);
	I_SimpleProperty(int, EnabledOrbiter, 
	                 __int__getEnabledOrbiter, 
	                 0);
	I_SimpleProperty(int, EnabledOscillator, 
	                 __int__getEnabledOscillator, 
	                 0);
	I_SimpleProperty(int, ExplosionDebugView, 
	                 __int__getExplosionDebugView, 
	                 __void__setExplosionDebugView__int);
	I_SimpleProperty(float, ExplosionEpsilon, 
	                 0, 
	                 __void__setExplosionEpsilon__float);
	I_SimpleProperty(float, ExplosionMagnitude, 
	                 0, 
	                 __void__setExplosionMagnitude__float);
	I_SimpleProperty(float, ExplosionRadius, 
	                 0, 
	                 __void__setExplosionRadius__float);
	I_SimpleProperty(float, ExplosionSigma, 
	                 0, 
	                 __void__setExplosionSigma__float);
	I_SimpleProperty(std::string, ExplosionTarget, 
	                 __std_string__getExplosionTarget, 
	                 0);
	I_SimpleProperty(float, FluidDensity, 
	                 0, 
	                 __void__setFluidDensity__float);
	I_SimpleProperty(float, FluidViscosity, 
	                 0, 
	                 __void__setFluidViscosity__float);
	I_SimpleProperty(osg::Vec2, FrequencyRange, 
	                 __osg_Vec2__getFrequencyRange, 
	                 0);
	I_SimpleProperty(const char *, Image, 
	                 0, 
	                 __void__setImage__C5_char_P1);
	I_SimpleProperty(const char *, ImagePath, 
	                 __C5_char_P1__getImagePath, 
	                 __void__setImagePath__C5_char_P1);
	I_SimpleProperty(float, LifeTime, 
	                 __float__getLifeTime, 
	                 __void__setLifeTime__float);
	I_SimpleProperty(int, Lighting, 
	                 __int__getLighting, 
	                 __void__setLighting__int);
	I_SimpleProperty(float, Mass, 
	                 __float__getMass, 
	                 __void__setMass__float);
	I_SimpleProperty(float, OrbitEpsilon, 
	                 0, 
	                 __void__setOrbitEpsilon__float);
	I_SimpleProperty(float, OrbitMagnitude, 
	                 0, 
	                 __void__setOrbitMagnitude__float);
	I_SimpleProperty(float, OrbitMaxRadius, 
	                 0, 
	                 __void__setOrbitMaxRadius__float);
	I_SimpleProperty(float, OscillatorAmplitude, 
	                 0, 
	                 __void__setOscillatorAmplitude__float);
	I_SimpleProperty(float, OscillatorFrequency, 
	                 0, 
	                 __void__setOscillatorFrequency__float);
	I_SimpleProperty(int, OscillatorLockAngle, 
	                 0, 
	                 __void__setOscillatorLockAngle__int);
	I_SimpleProperty(int, ParticleShape, 
	                 __int__getParticleShape, 
	                 __void__setParticleShape__int);
	I_SimpleProperty(int, PlacerType, 
	                 __int__getPlacerType, 
	                 __void__setPlacerType__int);
	I_SimpleProperty(osg::Vec2, RadialPhiRange, 
	                 __osg_Vec2__getRadialPhiRange, 
	                 0);
	I_SimpleProperty(osg::Vec2, RadialRange, 
	                 __osg_Vec2__getRadialRange, 
	                 0);
	I_SimpleProperty(float, Radius, 
	                 __float__getRadius, 
	                 __void__setRadius__float);
	I_SimpleProperty(osg::Vec2, ShooterPhiRange, 
	                 __osg_Vec2__getShooterPhiRange, 
	                 0);
	I_SimpleProperty(osg::Vec2, ShooterSpeedRange, 
	                 __osg_Vec2__getShooterSpeedRange, 
	                 0);
	I_SimpleProperty(osg::Vec2, ShooterThetaRange, 
	                 __osg_Vec2__getShooterThetaRange, 
	                 0);
	I_SimpleProperty(std::vector< lo_message >, State, 
	                 __std_vectorT1_lo_message___getState, 
	                 0);
	I_SimpleProperty(osg::Vec3, Translation, 
	                 __osg_Vec3__getTranslation, 
	                 0);
	I_SimpleProperty(int, UseShaders, 
	                 __int__getUseShaders, 
	                 __void__setUseShaders__int);
END_REFLECTOR

