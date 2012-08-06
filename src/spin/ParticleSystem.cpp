// -----------------------------------------------------------------------------
// |    ___  ___  _  _ _     ___                                        _      |
// |   / __>| . \| || \ |   | __>_ _  ___ ._ _ _  ___  _ _ _  ___  _ _ | |__   |
// |   \__ \|  _/| ||   |   | _>| '_><_> || ' ' |/ ._>| | | |/ . \| '_>| / /   |
// |   <___/|_|  |_||_\_|   |_| |_|  <___||_|_|_|\___.|__/_/ \___/|_|  |_\_\   |
// |                                                                           |
// |---------------------------------------------------------------------------|
//
// http://spinframework.sourceforge.net
// Copyright (C) 2009 Mike Wozniewski, Zack Settel
//
// Developed/Maintained by:
//    Mike Wozniewski (http://www.mikewoz.com)
//    Zack Settel (http://www.sheefa.net/zack)
//
// Principle Partners:
//    Shared Reality Lab, McGill University (http://www.cim.mcgill.ca/sre)
//    La Societe des Arts Technologiques (http://www.sat.qc.ca)
//
// Funding by:
//    NSERC/Canada Council for the Arts - New Media Initiative
//    Heritage Canada
//    Ministere du Developpement economique, de l'Innovation et de l'Exportation
//
// -----------------------------------------------------------------------------
//  This file is part of the SPIN Framework.
//
//  SPIN Framework is free software: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  SPIN Framework is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  along with SPIN Framework. If not, see <http://www.gnu.org/licenses/>.
// -----------------------------------------------------------------------------

#include <osg/Texture2D>
#include <osg/PointSprite>
#include <osg/BlendFunc>
#include <osgDB/FileUtils>
#include <osgDB/ReadFile>

#include "ParticleSystem.h"
#include "SceneManager.h"
#include "spinApp.h"
#include "spinBaseContext.h"
#include "osgUtil.h"

#define USE_LOCAL_SHADERS

namespace spin
{

// *****************************************************************************
// constructor:
ParticleSystem::ParticleSystem (SceneManager *sceneManager, const char* initID) : ConstraintsNode(sceneManager, initID)
{
	this->setName(this->getID() + ".ParticleSystem");
	this->setNodeType("ParticleSystem");

    imgPath_.clear();
    attachedFlag_ = false;
    emissive_ = false;
    lighting_ = false;
    
    lifetime_ = 3; // seconds
    radius_ = 0.05;
    mass_ = 0.05;

    freqRange_ = osg::Vec2(10.0, 30.0); // 10-30 particles/second
    angularRange_ = osg::Vec2(0.0, 2*osg::PI); // full circle
    speedRange_ = osg::Vec2(1.0, 2.0); // speed of 1-2 m/s
    alphaRange_ = osg::Vec2(0.0, 1.5); // alpha
    sizeRange_ = osg::Vec2(0.25, 1.0); // size of 25cm to 1m
    colorMin_ = osg::Vec4(1,1,1,1); // full white
    colorMax_ = osg::Vec4(1,0,0,1); // full red

    // First of all, we create the ParticleSystem object; it will hold
    // our particles and expose the interface for managing them; this object
    // is a Drawable, so we'll have to add it to a Geode later.
    system_ = new osgParticle::ParticleSystem();
    
    
    // As for other Drawable classes, the aspect of graphical elements of
    // ParticleSystem (the particles) depends on the StateAttribute's we
    // give it. The ParticleSystem class has an helper function that let
    // us specify a set of the most common attributes: setDefaultAttributes().
    // This method can accept up to three parameters; the first is a texture
    // name (std::string), which can be empty to disable texturing, the second
    // sets whether particles have to be "emissive" (additive blending) or not;
    // the third parameter enables or disables lighting.

    system_->setDefaultAttributes(imgPath_, emissive_, lighting_);


    // use a particle template:
    particle_.setLifeTime(lifetime_);
    particle_.setRadius(radius_);
    particle_.setMass(mass_);
    
    system_->setDefaultParticleTemplate(particle_);


    // Now that our particle system is set we have to create an emitter, that is
    // an object (actually a Node descendant) that generate new particles at 
    // each frame. The best choice is to use a ModularEmitter, which allow us to
    // achieve a wide variety of emitting styles by composing the emitter using
    // three objects: a "counter", a "placer" and a "shooter". The counter must
    // tell the ModularEmitter how many particles it has to create for the
    // current frame; then, the ModularEmitter creates these particles, and for
    // each new particle it instructs the placer and the shooter to set its
    // position vector and its velocity vector, respectively.
    // By default, a ModularEmitter object initializes itself with a counter of
    // type RandomRateCounter, a placer of type PointPlacer and a shooter of
    // type RadialShooter (see documentation for details). We are going to leave
    // these default objects there, but we'll modify the counter so that it
    // counts faster (more particles are emitted at each frame).

    emitter_ = new osgParticle::ModularEmitter;
    emitter_->setName(getID()+".ParticleEmitter");
    
    // the first thing you *MUST* do after creating an emitter is to set the
    // destination particle system, otherwise it won't know where to create
    // new particles.

    emitter_->setParticleSystem(system_);
    
    
    // set the counter:
    counter_ = new osgParticle::RandomRateCounter;
    counter_->setRateRange(freqRange_.x(), freqRange_.y());
    emitter_->setCounter(counter_);
    

    // set the placer:
    radialPlacer_ = new osgParticle::SectorPlacer;
    radialPlacer_->setCenter(0,0,0);
    radialPlacer_->setRadiusRange(0, 0);
    radialPlacer_->setPhiRange(0, 2 * osg::PI);
    emitter_->setPlacer(radialPlacer_);

    cubicPlacer_ = new osgParticle::BoxPlacer;
    cubicPlacer_->setXRange(-1,1);
    cubicPlacer_->setYRange(-1,1);
    cubicPlacer_->setZRange(-1,1);
    
    linearPlacer_ = new osgParticle::MultiSegmentPlacer;
    linearPlacer_->addVertex(-1,0,0);
    linearPlacer_->addVertex(1,0,0);
    


    // create the shooter:
    shooter_ = new osgParticle::RadialShooter;
    shooter_->setInitialSpeedRange(5, 10);
    emitter_->setShooter(shooter_);
    
    
    // The emitter is done! Let's attach it:
    this->getAttachmentNode()->addChild(emitter_);
    

    // To simulate the effect of the earth gravity we have to create a Program.
    // This is a particle processor just like the Emitter class, but it allows
    // us to modify particle properties *after* they have been created.
    // 
    // The ModularProgram class can be thought as a sequence of operators,
    // each one performing some actions on the particles. So, the trick is:
    // create the ModularProgram object, create one or more Operator objects,
    // add those operators to the ModularProgram, and finally add the
    // ModularProgram object to the scene graph.
    // NOTE: since the Program objects perform actions after the particles
    // have been emitted by one or more Emitter objects, all instances of
    // Program (and its descendants) should be placed *after* the instances
    // of Emitter objects in the scene graph.
    
    program_ = new osgParticle::ModularProgram;
    program_->setParticleSystem(system_);

    // Create operators that act on the particles.
    // (note: for now none of them are added to the program)
    opOrbit_ = new osgParticle::OrbitOperator;
    opAccel_ = new osgParticle::AccelOperator;
    opAngularAccel_ = new osgParticle::AngularAccelOperator;
    opAngularDamping_ = new osgParticle::AngularDampingOperator;
    opDamping_ = new osgParticle::DampingOperator;
    opFluidFriction_ = new osgParticle::FluidFrictionOperator;
    opExplosion_ = new osgParticle::ExplosionOperator;
    opForce_ = new osgParticle::ForceOperator;
    opBounce_ = new osgParticle::BounceOperator;

    // some intial settings for operators:
    opAccel_->setToGravity();
    opFluidFriction_->setFluidToWater();
    

    // add the program to the scene graph
    this->getAttachmentNode()->addChild(program_);
    
    // all we still need is
    // to create a Geode and add the particle system to it, so it can be
    // displayed.

    osg::Geode *geode = new osg::Geode;
    geode->setName(getID()+".ParticleSystem");
    geode->addDrawable(system_);

    // add the geode to the scene graph
    this->getAttachmentNode()->addChild(geode);
    
    
    updater_ = new osgParticle::ParticleSystemUpdater;
    updater_->setName(getID()+".ParticleUpdater");
    updater_->addParticleSystem(system_);
    //this->getAttachmentNode()->addChild(updater_);


        
    // A particle system and its placer has no osg::Group, so we create a fake
    // node that set as our attachmentNode, and we copy the position of the 
    // placer whenever it changes.
    attachPAT = new osg::PositionAttitudeTransform();
    this->addChild(attachPAT);
    setAttachmentNode(attachPAT.get());

}

// *****************************************************************************
// destructor
ParticleSystem::~ParticleSystem()
{

}


// *****************************************************************************
void ParticleSystem::callbackUpdate(osg::NodeVisitor* nv)
{
    ConstraintsNode::callbackUpdate(nv);
    
    if (!attachedFlag_)
    {
        this->getAttachmentNode()->addChild(updater_);
        attachedFlag_ = true;
    }
}

void ParticleSystem::updateNodePath(bool updateChildren)
{
	ConstraintsNode::updateNodePath(false);
    currentNodePath_.push_back(attachPAT.get());
    updateChildNodePaths();
}

    
// *****************************************************************************


void ParticleSystem::debug()
{
    ConstraintsNode::debug();
    std::cout << "   center: " << stringify(radialPlacer_->getCenter()) << std::endl;
}

void ParticleSystem::setTranslation (float x, float y, float z)
{
    //ConstraintsNode::setTranslation(x,y,z);
    radialPlacer_->setCenter(x,y,z);
    cubicPlacer_->setCenter(x,y,z);
    //attachPAT->setPosition(osg::Vec3(x,y,z));
    if (!broadcastLock_)
        BROADCAST(this, "sfff", "setTranslation", x, y, z);
}

void ParticleSystem::setOrientation (float p, float r, float y)
{
    ConstraintsNode::setOrientation(p,r,y);

}

void ParticleSystem::setOrientationQuat (float x, float y, float z, float w)
{
    ConstraintsNode::setOrientationQuat(x,y,z,w);

}

// *****************************************************************************
// placer properties:

void ParticleSystem::setPlacerType(int type)
{
    switch ((PlacerType)type)
    {
        default:
        case RADIAL:
            emitter_->setPlacer(radialPlacer_);
            break;
        case CUBIC:
            emitter_->setPlacer(cubicPlacer_);
            break;
        case LINEAR:
            emitter_->setPlacer(linearPlacer_);
            break;
    }
    BROADCAST(this, "si", "setPlacerType", type);
}

void ParticleSystem::setRadialRange(float min, float max)
{
    radialPlacer_->setRadiusRange(min, max);
    BROADCAST(this, "sff", "setRadialRange", min, max);
}
osg::Vec2 ParticleSystem::getRadialRange()
{
    const osgParticle::rangef range = radialPlacer_->getRadiusRange();
    return osg::Vec2(range.minimum, range.maximum);
}

void ParticleSystem::setRadialPhiRange(float min, float max)
{
    radialPlacer_->setPhiRange(min, max);
    BROADCAST(this, "sff", "setRadialPhiRange", min, max);
}

void ParticleSystem::setCubicXRange(float min, float max)
{
    cubicPlacer_->setXRange(min, max);
    BROADCAST(this, "sff", "setCubicXRange", min, max);
}

void ParticleSystem::setCubicYRange(float min, float max)
{
    cubicPlacer_->setYRange(min, max);
    BROADCAST(this, "sff", "setCubicYRange", min, max);
}

void ParticleSystem::setCubicZRange(float min, float max)
{
    cubicPlacer_->setZRange(min, max);
    BROADCAST(this, "sff", "setCubicZRange", min, max);
}

void ParticleSystem::addLinearPlacerVertex(float x, float y, float z)
{
    linearPlacer_->addVertex(osg::Vec3(x,y,z));
    BROADCAST(this, "sfff", "addLinearPlacerVertex", x, y, z);
}

void ParticleSystem::removeLinearPlacerVertices()
{
    while (linearPlacer_->numVertices())
    {
        linearPlacer_->removeVertex(0);
    }
    BROADCAST(this, "s", "removeLinearPlacerVertices");
}



// *****************************************************************************

void ParticleSystem::removeOperator(osgParticle::Operator *op)
{
    for (int i=0; i<program_->numOperators(); i++)
    {
        if (op==program_->getOperator(i))
        {
            program_->removeOperator(i);
            break;
        }
    }
}

void ParticleSystem::enableOrbiter(int b)
{
    if (b) program_->addOperator(opOrbit_.get());
    else removeOperator(opOrbit_.get());
    BROADCAST(this, "si", "enableOrbiter", b);
}

void ParticleSystem::enableAccelerator(int b)
{
    if (b) program_->addOperator(opAccel_.get());
    else removeOperator(opAccel_.get());
    BROADCAST(this, "si", "enableAccelerator", b);
}

void ParticleSystem::enableAngularAccelerator(int b)
{
    if (b) program_->addOperator(opAngularAccel_.get());
    else removeOperator(opAngularAccel_.get());
    BROADCAST(this, "si", "enableAngularAccelerator", b);
}

void ParticleSystem::enableAngularDamping(int b)
{
    if (b) program_->addOperator(opAngularDamping_.get());
    else removeOperator(opAngularDamping_.get());
    BROADCAST(this, "si", "enableAngularDamping", b);
}

void ParticleSystem::enableDamping(int b)
{
    if (b) program_->addOperator(opDamping_.get());
    else removeOperator(opDamping_.get());
    BROADCAST(this, "si", "enableDamping", b);
}

void ParticleSystem::enableFluidFriction(int b)
{
    if (b) program_->addOperator(opFluidFriction_.get());
    else removeOperator(opFluidFriction_.get());
    BROADCAST(this, "si", "enableFluidFriction", b);
}

void ParticleSystem::enableExplosion(int b)
{
    if (b) program_->addOperator(opExplosion_.get());
    else removeOperator(opExplosion_.get());
    BROADCAST(this, "si", "enableExplosion", b);
}

void ParticleSystem::enableForce(int b)
{
    if (b) program_->addOperator(opForce_.get());
    else removeOperator(opForce_.get());
    BROADCAST(this, "si", "enableForce", b);
}

void ParticleSystem::enableBouncer(int b)
{
    if (b) program_->addOperator(opBounce_.get());
    else removeOperator(opBounce_.get());
    BROADCAST(this, "si", "enableBouncer", b);
}



// *****************************************************************************
// operator properties:

void ParticleSystem::setAccel(float x, float y, float z)
{
    opAccel_->setAcceleration(osg::Vec3(x,y,z));
    BROADCAST(this, "sfff", "setAccel", x, y, z);
}

void ParticleSystem::setAngularAccel(float x, float y, float z)
{
    opAngularAccel_->setAngularAcceleration(osg::Vec3(x,y,z));
    BROADCAST(this, "sfff", "setAngularAccel", x, y, z);
}

void ParticleSystem::setAngularDamping(float d)
{
    opAngularDamping_->setDamping(d);
    BROADCAST(this, "sf", "setAngularDamping", d);
}

void ParticleSystem::setDamping(float d)
{
    opDamping_->setDamping(d);
    BROADCAST(this, "sf", "setDamping", d);
}

void ParticleSystem::setOrbitCenter(float x, float y, float z)
{
    opOrbit_->setCenter(osg::Vec3(x,y,z));
    BROADCAST(this, "sfff", "setOrbitCenter", x, y, z);
}
void ParticleSystem::setOrbitMagnitude(float mag)
{
    opOrbit_->setMagnitude(mag);
    BROADCAST(this, "sf", "setOrbitMagnitude", mag);
}
void ParticleSystem::setOrbitEpsilon(float eps)
{
    opOrbit_->setEpsilon(eps);
    BROADCAST(this, "sf", "setOrbitEpsilon", eps);
}
void ParticleSystem::setOrbitMaxRadius(float max)
{
    opOrbit_->setMaxRadius(max);
    BROADCAST(this, "sf", "setOrbitMaxRadius", max);
}


void ParticleSystem::setExplosionCenter(float x, float y, float z)
{
    opExplosion_->setCenter(osg::Vec3(x,y,z));
    BROADCAST(this, "sfff", "setExplosionCenter", x, y, z);
}
void ParticleSystem::setExplosionRadius(float r)
{
    opExplosion_->setRadius(r);
    BROADCAST(this, "sf", "setExplosionRadius", r);
}
void ParticleSystem::setExplosionMagnitude(float mag)
{
    opExplosion_->setMagnitude(mag);
    BROADCAST(this, "sf", "setExplosionMagnitude", mag);
}
void ParticleSystem::setExplosionEpsilon(float eps)
{
    opExplosion_->setEpsilon(eps);
    BROADCAST(this, "sf", "setExplosionEpsilon", eps);
}
void ParticleSystem::setExplosionSigma(float s)
{
    opExplosion_->setSigma(s);
    BROADCAST(this, "sf", "setExplosionSigma", s);
}

void ParticleSystem::setForce(float x, float y, float z)
{
    opForce_->setForce(osg::Vec3(x,y,z));
    BROADCAST(this, "sfff", "setForce", x, y, z);
}

// *****************************************************************************

void ParticleSystem::setFluidDensity(float d)
{
    opFluidFriction_->setFluidDensity(d);
    BROADCAST(this, "sf", "setFluidDensity", d);
}

void ParticleSystem::setFluidViscosity(float v)
{
    opFluidFriction_->setFluidViscosity(v);
    BROADCAST(this, "sf", "setFluidViscosity", v);
}

void ParticleSystem::setFluidDirection(float x, float y, float z)
{
    opFluidFriction_->setWind(osg::Vec3(x,y,z));
    BROADCAST(this, "sfff", "setFluidDirection", x,y,z);
}


// *****************************************************************************
void ParticleSystem::addBouncePlane(float normalX, float normalY, float normalZ, float x, float y, float z)
{
    opBounce_->addPlaneDomain(osg::Plane(osg::Vec3(normalX,normalY,normalZ), osg::Vec3(x,y,z)));
    BROADCAST(this, "sffffff", "addBouncePlane", normalX,normalY,normalZ, x,y,z);
}

void ParticleSystem::addBounceBox(float minX, float minY, float minZ, float maxX, float maxY, float maxZ)
{
    opBounce_->addBoxDomain(osg::Vec3(minX,minY,minZ), osg::Vec3(maxX,maxY,maxZ));
    BROADCAST(this, "sffffff", "addBounceBox", minX,minY,minZ, maxX,maxY,maxZ);
}

void ParticleSystem::addBounceSphere(float x, float y, float z, float radius)
{
    opBounce_->addSphereDomain(osg::Vec3(x,y,z), radius);
    BROADCAST(this, "sffff", "addBounceSphere", x,y,z,radius);
}

void ParticleSystem::removeAllBouncers()
{
    opBounce_->removeAllDomains();
    BROADCAST(this, "s", "removeAllBouncers");
}

void ParticleSystem::setBounceFriction(float f)
{
    opBounce_->setFriction(f);
    BROADCAST(this, "sf", "setBounceFriction");
}

void ParticleSystem::setBounceResilience(float r)
{
    opBounce_->setResilience(r);
    BROADCAST(this, "sf", "setBounceResilience");
}

void ParticleSystem::setBounceCutoff(float v)
{
    opBounce_->setCutoff(v);
    BROADCAST(this, "sf", "setBounceCutoff");
}


// *****************************************************************************






// *****************************************************************************
// particle properties:

void ParticleSystem::setParticleShape(int shp)
{
    particle_.setShape((osgParticle::Particle::Shape)shp);
    system_->setDefaultParticleTemplate(particle_);
    BROADCAST(this, "si", "setParticleShape", shp);
}

void ParticleSystem::setLifeTime(float seconds)
{
    particle_.setLifeTime(seconds);
    system_->setDefaultParticleTemplate(particle_);
    BROADCAST(this, "sf", "setLifeTime", getLifeTime());
}

void ParticleSystem::setRadius(float radius)
{
    particle_.setRadius(radius);
    system_->setDefaultParticleTemplate(particle_);
    BROADCAST(this, "sf", "setRadius", getRadius());
}

void ParticleSystem::setMass(float mass)
{
    particle_.setMass(mass);
    system_->setDefaultParticleTemplate(particle_);
    BROADCAST(this, "sf", "setMass", getMass());
}

void ParticleSystem::setParticleSizeRange(float x, float y)
{
    particle_.setSizeRange(osgParticle::rangef(x,y));
    system_->setDefaultParticleTemplate(particle_);
    BROADCAST(this, "sff", "setParticleSizeRange", x, y);
}
 

// *****************************************************************************
// shooter & counter properties:

void ParticleSystem::setFrequencyRange (float min, float max)
{
    freqRange_ = osg::Vec2(min,max);
    
    // make sure rate is above zero and that max is bigger (or equal) to min:
    if (freqRange_.x() < 0) freqRange_.x() = 0;
    if (freqRange_.y() < freqRange_.x()) freqRange_.y() = freqRange_.x();

    // Get a pointer to the emitter's Counter object:
    osgParticle::RandomRateCounter *rrc =
        static_cast<osgParticle::RandomRateCounter *>(emitter_->getCounter());

    // Set the min and max rate (number of particles per second). The actual
    // rate at each frame will be chosen randomly within that range.
    rrc->setRateRange(freqRange_.x(), freqRange_.y());

	BROADCAST(this, "sff", "setFrequencyRange", freqRange_.x(), freqRange_.y());
}

/*
This shooter computes the velocity vector of incoming particles by choosing a
random direction and a random speed. Both direction and speed are chosen within
specified ranges. The direction is defined by two angles: theta, which is the
angle between the velocity vector and the Z axis, and phi, which is the angle
between the X axis and the velocity vector projected onto the X-Y plane. 
*/

void ParticleSystem::setShooterThetaRange(float min, float max)
{
    shooter_->setThetaRange(min, max);
    BROADCAST(this, "sff", "setShooterThetaRange", min, max);
}

void ParticleSystem::setShooterPhiRange(float min, float max)
{
    shooter_->setPhiRange(min, max);
    BROADCAST(this, "sff", "setShooterPhiRange", min, max);
}

void ParticleSystem::setShooterSpeedRange(float min, float max)
{
    shooter_->setInitialSpeedRange(min, max);
    BROADCAST(this, "sff", "setShooterSpeedRange", min, max);
}

osg::Vec2 ParticleSystem::getShooterSpeedRange()
{
    const osgParticle::rangef range = shooter_->getInitialSpeedRange();
    return osg::Vec2(range.minimum, range.maximum);
}

void ParticleSystem::setShootertRotationalSpeedRange(float minX, float minY, float minZ, float maxX, float maxY, float maxZ)
{
    shooter_->setInitialRotationalSpeedRange(osg::Vec3(minX, minY, minZ), osg::Vec3(maxX, maxY, maxZ));
    BROADCAST(this, "sffffff", "setShooterRotationalSpeedRange", minX, minY, minZ, maxX, maxY, maxZ);
}


// *****************************************************************************

void ParticleSystem::setImagePath (const char* path)
{
    if (imgPath_ != path)
    {
        imgPath_ = path;
        if (imgPath_=="NULL")
            system_->setDefaultAttributesUsingShaders("", emissive_, lighting_);
        else 
            system_->setDefaultAttributesUsingShaders(imgPath_, emissive_, lighting_);
        
        BROADCAST(this, "ss", "setImagePath", getImagePath());
    }
}

void ParticleSystem::setEmissive (int b)
{
	if (emissive_ != (bool)b)
	{
		emissive_ = (bool) b;
		BROADCAST(this, "si", "setEmmisive", getEmissive());
	}
}

void ParticleSystem::setLighting (int b)
{
	if (lighting_ != (bool)b)
	{
		lighting_ = (bool) b;
		BROADCAST(this, "si", "setLighting", getLighting());
	}
}

void ParticleSystem::setImage (const char* path)
{
    if (imgPath_ == path) return;
    imgPath_ = path;

    int texture_unit = 0;
    
    osg::StateSet *stateset = new osg::StateSet;
    stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

    osg::PointSprite *sprite = new osg::PointSprite;
    stateset->setTextureAttributeAndModes(texture_unit, sprite, osg::StateAttribute::ON);

    #if !defined(OSG_GLES1_AVAILABLE) && !defined(OSG_GLES2_AVAILABLE)
        stateset->setMode(GL_VERTEX_PROGRAM_POINT_SIZE, osg::StateAttribute::ON);
    #else
        OSG_NOTICE<<"Warning: ParticleSystem::setDefaultAttributesUsingShaders(..) not fully implemented."<<std::endl;
    #endif

    if (imgPath_!="NULL")
    {
        osg::Texture2D *texture = new osg::Texture2D;
        texture->setImage(osgDB::readImageFile(imgPath_));
        texture->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR);
        texture->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR);
        texture->setWrap(osg::Texture2D::WRAP_S, osg::Texture2D::MIRROR);
        texture->setWrap(osg::Texture2D::WRAP_T, osg::Texture2D::MIRROR);
        stateset->setTextureAttributeAndModes(texture_unit, texture, osg::StateAttribute::ON);
    }

    osg::BlendFunc *blend = new osg::BlendFunc;
    if (emissive_)
    {
        blend->setFunction(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE);
    }
    else
    {
        blend->setFunction(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA);
    }
    stateset->setAttributeAndModes(blend, osg::StateAttribute::ON);

    osg::Program *program = new osg::Program;
#ifdef USE_LOCAL_SHADERS
    char vertexShaderSource[] =
        "uniform float visibilityDistance;\n"
        "varying vec3 basic_prop;\n"
        "\n"
        "void main(void)\n"
        "{\n"
        "    basic_prop = gl_MultiTexCoord0.xyz;\n"
        "    \n"
        "    vec4 ecPos = gl_ModelViewMatrix * gl_Vertex;\n"
        "    float ecDepth = -ecPos.z;\n"
        "    \n"
        "    if (visibilityDistance > 0.0)\n"
        "    {\n"
        "        if (ecDepth <= 0.0 || ecDepth >= visibilityDistance)\n"
        "            basic_prop.x = -1.0;\n"
        "    }\n"
        "    \n"
        "    gl_Position = ftransform();\n"
        "    gl_ClipVertex = ecPos;\n"
        "    \n"
        "    vec4 color = gl_Color;\n"
        "    color.a *= basic_prop.z;\n"
        "    gl_FrontColor = color;\n"
        "    gl_BackColor = gl_FrontColor;\n"
        "}\n";
    char fragmentShaderSource[] =
        "uniform sampler2D baseTexture;\n"
        "varying vec3 basic_prop;\n"
        "\n"
        "void main(void)\n"
        "{\n"
        "    if (basic_prop.x < 0.0) discard;\n"
        "    gl_FragColor = gl_Color * texture2D(baseTexture, gl_TexCoord[0].xy);\n"
        "}\n";
    program->addShader(new osg::Shader(osg::Shader::VERTEX, vertexShaderSource));
    program->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragmentShaderSource));
#else
    program->addShader(osg::Shader::readShaderFile(osg::Shader::VERTEX, osgDB::findDataFile("shaders/particle.vert")));
    program->addShader(osg::Shader::readShaderFile(osg::Shader::FRAGMENT, osgDB::findDataFile("shaders/particle.frag")));
#endif
    stateset->setAttributeAndModes(program, osg::StateAttribute::ON);

    stateset->addUniform(new osg::Uniform("visibilityDistance", (float)system_->getVisibilityDistance()));
    stateset->addUniform(new osg::Uniform("baseTexture", texture_unit));
    system_->setStateSet(stateset);

    system_->setUseVertexArray(true);
    system_->setUseShaders(true);
             
                                                                                                                                                                                                                                                             
    BROADCAST(this, "ss", "setImage", getImagePath());
}

// *****************************************************************************

std::vector<lo_message> ParticleSystem::getState () const
{
    // inherit state from base class
	std::vector<lo_message> ret = ConstraintsNode::getState();

    lo_message msg;

    msg = lo_message_new();
    lo_message_add(msg, "si", "setEmissive", getEmissive());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "si", "setLighting", getLighting());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "ss", "setImagePath", getImagePath());
    ret.push_back(msg);

    msg = lo_message_new();
    osg::Vec2 v = getFrequencyRange();
    lo_message_add(msg, "sff", "setFrequencyRange", v.x(), v.y());
    ret.push_back(msg);

    return ret;
}

} // end of namespace spin

