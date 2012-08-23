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
#include <osg/PolygonMode>
#include <osg/BlendFunc>
#include <osgDB/FileUtils>
#include <osgDB/ReadFile>

#include "ParticleSystem.h"
#include "SceneManager.h"
#include "ShapeNode.h"
#include "spinApp.h"
#include "spinBaseContext.h"
#include "osgUtil.h"

#define USE_LOCAL_SHADERS 1

extern pthread_mutex_t sceneMutex;

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
    //system_ = new osgParticle::ConnectedParticleSystem();
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
    opAccel_ = new osgParticle::AccelOperator;
    opAngularAccel_ = new osgParticle::AngularAccelOperator;
    opFluidFriction_ = new osgParticle::FluidFrictionOperator;
    opForce_ = new osgParticle::ForceOperator;

#ifdef OSG_MIN_VERSION_REQUIRED
#if OSG_MIN_VERSION_REQUIRED(3,0,0)
    opOrbit_ = new osgParticle::OrbitOperator;
    opAngularDamping_ = new osgParticle::AngularDampingOperator;
    opDamping_ = new osgParticle::DampingOperator;
    opExplosion_ = new osgParticle::ExplosionOperator;
    opBouncer_ = new BouncerOperator;
#else
    opOrbit_ = new NullOperator;
    opAngularDamping_ = new NullOperator;
    opDamping_ = new NullOperator;
    opExplosion_ = new NullOperator;
    opBouncer_ = new NullOperator;
#endif
#endif
    
    opAccel_->setName("ParticleSystem.opAccel");
    opAngularAccel_->setName("ParticleSystem.opAngularAccel");
    opFluidFriction_->setName("ParticleSystem.opFluidFriction");
    opForce_->setName("ParticleSystem.opForce");
    opOrbit_->setName("ParticleSystem.opOrbit");
    opAngularDamping_->setName("ParticleSystem.opAngularDamping");
    opDamping_->setName("ParticleSystem.opDamping");
    opExplosion_->setName("ParticleSystem.opExplosion");
    opBouncer_->setName("ParticleSystem.opBouncer");

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
    
    // Here, we update the bouncer domains based on the position and size of
    // the target node. For ShapeNode's we can do a good job in the case of a 
    // PLANE or BOX, but for everything else, we just approximate the shape of
    // the node with a SPHERE (updated from the BoundingSphere that is computed
    // anyway during the cull traversal).
    //
    // TODO: figure out how to update these things ONLY when they change. The
    // code below updates them every frame.
    int i = 0;
    std::vector< osg::observer_ptr<ReferencedNode> >::iterator it;
    for (it=bounceTargets_.begin(); it!=bounceTargets_.end(); ++it)
    {
        osgParticle::DomainOperator::Domain* d = opBouncer_->getDomainPointer(i);
        if ((*it).valid() && d)
        {
            ShapeNode *shp = dynamic_cast<ShapeNode*>((*it).get());
            if (shp && shp->getShape()==ShapeNode::PLANE)
            {
                d->type = osgParticle::DomainOperator::Domain::PLANE_DOMAIN;
                
                // TODO: use global translation and rotation:
                d->plane.set(osg::Plane(shp->getOrientationQuat()*osg::Y_AXIS,shp->getTranslation()));
                
                // this stuff tries to get the RECT_DOMAIN working:
                /*
                 d->type = osgParticle::DomainOperator::Domain::RECT_DOMAIN;
                d->v1 = shp->getTranslation() - (shp->getScale()/2); //corner
                d->v2 = shp->getOrientationQuat()*osg::X_AXIS*shp->getScale().x(); // w
                d->v3 = shp->getOrientationQuat()*osg::Z_AXIS*shp->getScale().z(); // h
                
                //d->v2 = osg::X_AXIS * shp->getScale().x(); // w
                //d->v3 = osg::Z_AXIS * shp->getScale().z(); // h
              
                opBouncer_->updatePlane(d);
                */
            }
            else if (shp && shp->getShape()==ShapeNode::BOX)
            {
                d->type = osgParticle::DomainOperator::Domain::BOX_DOMAIN;
                // min coord:
                d->v1 = shp->getTranslation() - (shp->getScale()/2);
                // max:
                d->v2 = shp->getTranslation() + (shp->getScale()/2);
            }
            else if (shp && shp->getShape()==ShapeNode::DISC)
            {
                d->type = osgParticle::DomainOperator::Domain::DISK_DOMAIN;
                d->v1 = shp->getTranslation();
                d->v2 = shp->getOrientationQuat() * osg::Y_AXIS;
                // we have to approximate the radius by taking the average of 
                // the x and z scale (will be correct if uniform scale):
                d->r1 = (shp->getScale().x()+shp->getScale().z()) / 4;
                d->r2 = 0;
                d->plane.set(d->v2, d->v1);
            }
            else
            {
                d->type = osgParticle::DomainOperator::Domain::SPHERE_DOMAIN;
                osg::BoundingSphere bound = (*it)->getBound();
                d->v1 = bound.center();
                d->r1 = bound.radius();
            }
        }
        // if the target has been removed, we need to remove it from our list:
        else
        {
            if (!(*it).valid()) bounceTargets_.erase(it);
            if (d) opBouncer_->removeDomain(i);
            break;
        }
        
        i++;
    }
    
    if (opExplosionTarget_.valid())
    {
        opExplosion_->setCenter(opExplosionTarget_->getCenter());
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
    
    std::cout << "   " << bounceTargets_.size() << " Bouncer targets:" << std::endl;
    for (unsigned int i=0; i<bounceTargets_.size(); i++)
    {
        std::cout << "   - " << bounceTargets_[i]->getID() << std::endl;
        osgParticle::DomainOperator::Domain* d = opBouncer_->getDomainPointer(i);
        std::cout << "     domain=" << d->type << std::endl;
        std::cout << "     v1=" << stringify(d->v1) << std::endl;
        std::cout << "     v2=" << stringify(d->v2) << std::endl;
        std::cout << "     v3=" << stringify(d->v3) << std::endl;
        std::cout << "     s1=" << stringify(d->v3) << std::endl;
        std::cout << "     s2=" << stringify(d->v3) << std::endl;
        std::cout << "     r1=" << d->r1 << std::endl;
        std::cout << "     r2=" << d->r2 << std::endl;
        std::cout << "     plane=" << stringify(d->plane.asVec4()) << std::endl;
        std::cout << "     planeNorm=" << stringify(d->plane.getNormal()) << std::endl;
    }
    
    std::cout << "   " << program_->numOperators() << " operators:" << std::endl;
    for (unsigned int i=0; i<program_->numOperators(); i++)
    {
        std::cout << "   - " << program_->getOperator(i)->getName() << std::endl;
    }
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

int ParticleSystem::getPlacerType() const
{
    if (emitter_->getPlacer() == cubicPlacer_)
        return (int)CUBIC;
    else if (emitter_->getPlacer() == linearPlacer_)
        return (int)LINEAR;
    else    
        return (int)RADIAL;
}

void ParticleSystem::setRadialRange(float min, float max)
{
    radialPlacer_->setRadiusRange(min, max);
    BROADCAST(this, "sff", "setRadialRange", min, max);
}
osg::Vec2 ParticleSystem::getRadialRange() const
{
    const osgParticle::rangef range = radialPlacer_->getRadiusRange();
    return osg::Vec2(range.minimum, range.maximum);
}

void ParticleSystem::setRadialPhiRange(float min, float max)
{
    radialPlacer_->setPhiRange(osg::DegreesToRadians(min), osg::DegreesToRadians(max));
    BROADCAST(this, "sff", "setRadialPhiRange", min, max);
}
osg::Vec2 ParticleSystem::getRadialPhiRange() const
{
    const osgParticle::rangef range = radialPlacer_->getPhiRange();
    return osg::Vec2(osg::RadiansToDegrees(range.minimum), osg::RadiansToDegrees(range.maximum));
}

void ParticleSystem::setCubicXRange(float min, float max)
{
    cubicPlacer_->setXRange(min, max);
    BROADCAST(this, "sff", "setCubicXRange", min, max);
}
osg::Vec2 ParticleSystem::getCubicXRange() const
{
    const osgParticle::rangef range = cubicPlacer_->getXRange();
    return osg::Vec2(range.minimum, range.maximum);
}

void ParticleSystem::setCubicYRange(float min, float max)
{
    cubicPlacer_->setYRange(min, max);
    BROADCAST(this, "sff", "setCubicYRange", min, max);
}
osg::Vec2 ParticleSystem::getCubicYRange() const
{
    const osgParticle::rangef range = cubicPlacer_->getYRange();
    return osg::Vec2(range.minimum, range.maximum);
}

void ParticleSystem::setCubicZRange(float min, float max)
{
    cubicPlacer_->setZRange(min, max);
    BROADCAST(this, "sff", "setCubicZRange", min, max);
}
osg::Vec2 ParticleSystem::getCubicZRange() const
{
    const osgParticle::rangef range = cubicPlacer_->getZRange();
    return osg::Vec2(range.minimum, range.maximum);
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
    if (b && !getEnabledOrbiter())
        program_->addOperator(opOrbit_.get());
    else if (!b)
        for (unsigned int i=0; i<program_->numOperators(); i++)
        {
            if (program_->getOperator(i) == opOrbit_.get())
                removeOperator(opOrbit_.get());
        }
    BROADCAST(this, "si", "enableOrbiter", b);
}
int ParticleSystem::getEnabledOrbiter() const
{
    for (int i=0; i<program_->numOperators(); i++)
        if (program_->getOperator(i)==opOrbit_) return 1;
    return 0;
}

void ParticleSystem::enableAccelerator(int b)
{
    if (b && !getEnabledAccelerator())
        program_->addOperator(opAccel_.get());
    else if (!b)
        for (unsigned int i=0; i<program_->numOperators(); i++)
        {
            if (program_->getOperator(i) == opAccel_.get())
                removeOperator(opAccel_.get());
        }

    BROADCAST(this, "si", "enableAccelerator", b);
}
int ParticleSystem::getEnabledAccelerator() const
{
    for (int i=0; i<program_->numOperators(); i++)
        if (program_->getOperator(i)==opAccel_) return 1;
    return 0;
}

void ParticleSystem::enableAngularAccelerator(int b)
{
    if (b && !getEnabledAngularAccelerator())
        program_->addOperator(opAngularAccel_.get());
    else if (!b)
        for (unsigned int i=0; i<program_->numOperators(); i++)
        {
            if (program_->getOperator(i) == opAngularAccel_.get())
                removeOperator(opAngularAccel_.get());
        }
    BROADCAST(this, "si", "enableAngularAccelerator", b);
}
int ParticleSystem::getEnabledAngularAccelerator() const
{
    for (int i=0; i<program_->numOperators(); i++)
        if (program_->getOperator(i)==opAngularAccel_) return 1;
    return 0;
}

void ParticleSystem::enableAngularDamping(int b)
{
    if (b && !getEnabledAngularDamping())
        program_->addOperator(opAngularDamping_.get());
    else if (!b)
        for (unsigned int i=0; i<program_->numOperators(); i++)
        {
            if (program_->getOperator(i) == opAngularDamping_.get())
                removeOperator(opAngularDamping_.get());
        }
    BROADCAST(this, "si", "enableAngularDamping", b);
}
int ParticleSystem::getEnabledAngularDamping() const
{
    for (int i=0; i<program_->numOperators(); i++)
        if (program_->getOperator(i)==opAngularDamping_) return 1;
    return 0;
}

void ParticleSystem::enableDamping(int b)
{
    if (b && !getEnabledDamping())
        program_->addOperator(opDamping_.get());
    else if (!b)
        for (unsigned int i=0; i<program_->numOperators(); i++)
        {
            if (program_->getOperator(i) == opDamping_.get())
                removeOperator(opDamping_.get());
        }
    BROADCAST(this, "si", "enableDamping", b);
}
int ParticleSystem::getEnabledDamping() const
{
    for (int i=0; i<program_->numOperators(); i++)
        if (program_->getOperator(i)==opDamping_) return 1;
    return 0;
}

void ParticleSystem::enableFluidFriction(int b)
{
    if (b && !getEnabledFluidFriction())
        program_->addOperator(opFluidFriction_.get());
    else if (!b)
        for (unsigned int i=0; i<program_->numOperators(); i++)
        {
            if (program_->getOperator(i) == opFluidFriction_.get())
                removeOperator(opFluidFriction_.get());
        }
    BROADCAST(this, "si", "enableFluidFriction", b);
}
int ParticleSystem::getEnabledFluidFriction() const
{
    for (int i=0; i<program_->numOperators(); i++)
        if (program_->getOperator(i)==opFluidFriction_) return 1;
    return 0;
}

void ParticleSystem::enableExplosion(int b)
{
    if (b && !getEnabledExplosion())
        program_->addOperator(opExplosion_.get());
    else if (!b)
        for (unsigned int i=0; i<program_->numOperators(); i++)
        {
            if (program_->getOperator(i) == opExplosion_.get())
                removeOperator(opExplosion_.get());
        }
    BROADCAST(this, "si", "enableExplosion", b);
}
int ParticleSystem::getEnabledExplosion() const
{
    for (int i=0; i<program_->numOperators(); i++)
        if (program_->getOperator(i)==opExplosion_) return 1;
    return 0;
}

void ParticleSystem::enableForce(int b)
{
    if (b && !getEnabledForce())
        program_->addOperator(opForce_.get());
    else if (!b)
        for (unsigned int i=0; i<program_->numOperators(); i++)
        {
            if (program_->getOperator(i) == opForce_.get())
                removeOperator(opForce_.get());
        }
    BROADCAST(this, "si", "enableForce", b);
}
int ParticleSystem::getEnabledForce() const
{
    for (int i=0; i<program_->numOperators(); i++)
        if (program_->getOperator(i)==opForce_) return 1;
    return 0;
}

void ParticleSystem::enableBouncer(int b)
{
    if (b && !getEnabledBouncer())
        program_->addOperator(opBouncer_.get());
    else if (!b)
        for (unsigned int i=0; i<program_->numOperators(); i++)
        {
            if (program_->getOperator(i) == opBouncer_.get())
                removeOperator(opBouncer_.get());
        }
    BROADCAST(this, "si", "enableBouncer", b);
}
int ParticleSystem::getEnabledBouncer() const
{
    for (int i=0; i<program_->numOperators(); i++)
        if (program_->getOperator(i)==opBouncer_) return 1;
    return 0;
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

void ParticleSystem::setExplosionTarget(const char* targetID)
{
    GroupNode* n = dynamic_cast<GroupNode*>(sceneManager_->getNode(targetID));
    if (n)
    {
        opExplosionTarget_ = n;
    }
    else
    {
        std::cout << "ParticleSystem Error: Could not setExplosionTarget to '" << targetID << "' because that node could not be found" << std::endl;
    }

    BROADCAST(this, "ss", "setExplosionTarget", getExplosionTarget().c_str());
}
    
void ParticleSystem::setExplosionDebugView(int b)
{
    if (b)
    {
        // if this is the first time this is turned on, we need to create it:
        if (!opExplosionDebugView_.valid())
        {
            opExplosionDebugView_ = new osg::PositionAttitudeTransform();
            opExplosionDebugView_->setPosition(opExplosion_->getCenter());
            opExplosionDebugView_->setScale(osg::Vec3(1,1,1)*opExplosion_->getRadius());
            
            osg::TessellationHints* hints = new osg::TessellationHints;
            hints->setDetailRatio(0.5);
            osg::Geode *geode = new osg::Geode();
            geode->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.0f,0.0f,0.0f),1), hints));
            
            // wireframe:
            osg::PolygonMode* polygonMode = new osg::PolygonMode;
            polygonMode->setMode( osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE );
            geode->getOrCreateStateSet()->setAttributeAndModes( polygonMode, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON );
            
            // disable lighting:
            geode->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
            
            opExplosionDebugView_->addChild(geode);
        }
        
        if (!this->containsNode(opExplosionDebugView_.get()))
        {
            pthread_mutex_lock(&sceneMutex);
            this->addChild(opExplosionDebugView_.get());
            pthread_mutex_unlock(&sceneMutex);
        }
    }
    else
    {
        if (this->containsNode(opExplosionDebugView_.get()))
        {
            pthread_mutex_lock(&sceneMutex);
            this->removeChild(opExplosionDebugView_.get());
            pthread_mutex_unlock(&sceneMutex);
        }
    }
            
    BROADCAST(this, "si", "setExplosionDebugView", b);
}
    
void ParticleSystem::setExplosionCenter(float x, float y, float z)
{
    opExplosion_->setCenter(osg::Vec3(x,y,z));
    if (opExplosionDebugView_.valid())
        opExplosionDebugView_->setPosition(osg::Vec3(x,y,z));
    BROADCAST(this, "sfff", "setExplosionCenter", x, y, z);
}
void ParticleSystem::setExplosionRadius(float r)
{
    opExplosion_->setRadius(r);
    if (opExplosionDebugView_.valid()) 
        opExplosionDebugView_->setScale(osg::Vec3(1,1,1)*r);
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
    opExplosion_->setSigma(osg::DegreesToRadians(s));
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
    
void ParticleSystem::addBounceTarget(const char* nodeID)
{
    ReferencedNode* n = sceneManager_->getNode(nodeID);
    if (!n)
    {
        std::cout << "ParticleSystem Error: Could not addBounceTarget because node '" << nodeID << "' was not found" << std::endl;
        return;
    }
    
    std::vector< osg::observer_ptr<ReferencedNode> >::iterator it;
    for (it=bounceTargets_.begin(); it!=bounceTargets_.end(); ++it)
    {
        if ((*it).get() == n)
        {
            std::cout << "ParticleSystem Error: Node '" << nodeID << "' already exists in bounce targets" << std::endl;
            return;
        }
    }
    
    bounceTargets_.push_back(n);
    opBouncer_->addPlaneDomain(osg::Plane(osg::Z_AXIS, osg::Vec3(0,0,0)));
    
    BROADCAST(this, "ss", "addBounceTarget", nodeID);
}
    
void ParticleSystem::removeBounceTarget(const char* nodeID)
{
    ReferencedNode* n = sceneManager_->getNode(nodeID);
    if (!n)
    {
        std::cout << "ParticleSystem Error: Could not removeBounceTarget because node '" << nodeID << "' was not found" << std::endl;
        return;
    }
        
    int i = 0;
    std::vector< osg::observer_ptr<ReferencedNode> >::iterator it;
    for (it=bounceTargets_.begin(); it!=bounceTargets_.end(); ++it)
    {
        if ((*it).get() == n)
        {
            bounceTargets_.erase(it);
            opBouncer_->removeDomain(i);
            return;
        }
    }
    
    BROADCAST(this, "ss", "removeBounceTarget", nodeID);
}

void ParticleSystem::removeAllBounceTargets()
{
    bounceTargets_.clear();
    opBouncer_->removeAllDomains();
    BROADCAST(this, "s", "removeAllBounceTargets");
}
    
void ParticleSystem::addBouncePlane(float normalX, float normalY, float normalZ, float x, float y, float z)
{
    opBouncer_->addPlaneDomain(osg::Plane(osg::Vec3(normalX,normalY,normalZ), osg::Vec3(x,y,z)));
    BROADCAST(this, "sffffff", "addBouncePlane", normalX,normalY,normalZ, x,y,z);
}

void ParticleSystem::addBounceBox(float minX, float minY, float minZ, float maxX, float maxY, float maxZ)
{
    opBouncer_->addBoxDomain(osg::Vec3(minX,minY,minZ), osg::Vec3(maxX,maxY,maxZ));
    BROADCAST(this, "sffffff", "addBounceBox", minX,minY,minZ, maxX,maxY,maxZ);
}

void ParticleSystem::addBounceSphere(float x, float y, float z, float radius)
{
    opBouncer_->addSphereDomain(osg::Vec3(x,y,z), radius);
    BROADCAST(this, "sffff", "addBounceSphere", x,y,z,radius);
}

void ParticleSystem::removeAllBouncers()
{
    opBouncer_->removeAllDomains();
    BROADCAST(this, "s", "removeAllBouncers");
}

void ParticleSystem::setBounceFriction(float f)
{
    opBouncer_->setFriction(f);
    BROADCAST(this, "sf", "setBounceFriction", f);
}

void ParticleSystem::setBounceResilience(float r)
{
    opBouncer_->setResilience(r);
    BROADCAST(this, "sf", "setBounceResilience", r);
}

void ParticleSystem::setBounceCutoff(float v)
{
    opBouncer_->setCutoff(v);
    BROADCAST(this, "sf", "setBounceCutoff", v);
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

void ParticleSystem::setParticleSizeRange(float min, float max)
{
    particle_.setSizeRange(osgParticle::rangef(min, max));
    system_->setDefaultParticleTemplate(particle_);
    BROADCAST(this, "sff", "setParticleSizeRange", min, max);
}
    
void ParticleSystem::setParticleAlphaRange(float min, float max)
{
    particle_.setAlphaRange(osgParticle::rangef(min, max));
    system_->setDefaultParticleTemplate(particle_);
    BROADCAST(this, "sff", "setParticleAlphaRange", min, max);
}    
 
void ParticleSystem::setParticleColorRange(float minR, float minG, float minB, float maxR, float maxG, float maxB)
{
    particle_.setColorRange(osgParticle::rangev4(osg::Vec4(minR, minG, minB, particle_.getAlphaRange().minimum), osg::Vec4(maxR, maxG, maxB, particle_.getAlphaRange().maximum)));
    system_->setDefaultParticleTemplate(particle_);
    BROADCAST(this, "sffffff", "setParticleColorRange", minR, minG, minB, maxR, maxG, maxB);
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
    shooter_->setThetaRange(osg::DegreesToRadians(min), osg::DegreesToRadians(max));
    BROADCAST(this, "sff", "setShooterThetaRange", min, max);
}
osg::Vec2 ParticleSystem::getShooterThetaRange() const
{
    const osgParticle::rangef range = shooter_->getThetaRange();
    return osg::Vec2(osg::RadiansToDegrees(range.minimum), osg::RadiansToDegrees(range.maximum));
}

void ParticleSystem::setShooterPhiRange(float min, float max)
{
    shooter_->setPhiRange(osg::DegreesToRadians(min), osg::DegreesToRadians(max));
    BROADCAST(this, "sff", "setShooterPhiRange", min, max);
}
osg::Vec2 ParticleSystem::getShooterPhiRange() const
{
    const osgParticle::rangef range = shooter_->getPhiRange();
    return osg::Vec2(osg::RadiansToDegrees(range.minimum), osg::RadiansToDegrees(range.maximum));
}
    
void ParticleSystem::setShooterSpeedRange(float min, float max)
{
    shooter_->setInitialSpeedRange(min, max);
    BROADCAST(this, "sff", "setShooterSpeedRange", min, max);
}
osg::Vec2 ParticleSystem::getShooterSpeedRange() const
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
    std::string newPath = getRelativePath(std::string(path));
    if (imgPath_ != newPath)
    {
        imgPath_ = newPath;
        if (imgPath_=="NULL")
        {
            if (getUseShaders())
                system_->setDefaultAttributesUsingShaders("", emissive_, lighting_);
            else
                system_->setDefaultAttributes("", emissive_, lighting_);
        }
        else 
        {
            if (getUseShaders())
                system_->setDefaultAttributesUsingShaders(getAbsolutePath(imgPath_), emissive_, lighting_);
            else
                system_->setDefaultAttributes(getAbsolutePath(imgPath_), emissive_, lighting_);
        }
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
    /*
	if (lighting_ != (bool)b)
	{
		lighting_ = (bool) b;
		BROADCAST(this, "si", "setLighting", getLighting());
	}
    */
    
    // forward message to stateset if there is one:
    osg::ref_ptr<ReferencedStateSet> ss = dynamic_cast<ReferencedStateSet*>(stateset_->s_thing);
    if (ss.valid())
        ss->setLighting(b);
    else
    {
        lighting_ = (bool)b;
        system_->setDefaultAttributes("", emissive_, lighting_);
        BROADCAST(this, "si", "setLighting", getLighting());
    }
}

void ParticleSystem::setUseShaders (int b)
{
    system_->setUseShaders((bool)b);
    BROADCAST(this, "si", "setUseShaders", getUseShaders());
}
    
void ParticleSystem::updateStateSet()
{
	osg::ref_ptr<ReferencedStateSet> ss = dynamic_cast<ReferencedStateSet*>(stateset_->s_thing);
	system_->setStateSet( ss.get() );
    
    
    // force some stuff that is needed by particle system:
    int texture_unit = 0;
    
    
    ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
 
    osg::PointSprite *sprite = new osg::PointSprite;
    ss->setTextureAttributeAndModes(texture_unit, sprite, osg::StateAttribute::ON);
   

    #if !defined(OSG_GLES1_AVAILABLE) && !defined(OSG_GLES2_AVAILABLE)
        ss->setMode(GL_VERTEX_PROGRAM_POINT_SIZE, osg::StateAttribute::ON);
    #else
        OSG_NOTICE<<"Warning: ParticleSystem::setDefaultAttributesUsingShaders(..) not fully implemented."<<std::endl;
    #endif
    
    osg::BlendFunc *blend = new osg::BlendFunc;
    if (emissive_)
    {
        blend->setFunction(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE);
    }
    else
    {
        blend->setFunction(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA);
    }
    ss->setAttributeAndModes(blend, osg::StateAttribute::ON);
    
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
    
    // shader temporarilty disabled:
    /*
    ss->setAttributeAndModes(program, osg::StateAttribute::ON);

    ss->addUniform(new osg::Uniform("visibilityDistance", (float)system_->getVisibilityDistance()));
    ss->addUniform(new osg::Uniform("baseTexture", texture_unit));
    
    system_->setUseVertexArray(true);
    system_->setUseShaders(true);
    */
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
    
    osg::Vec2 v2;
    osg::Vec3 v3;
    osgParticle::rangef range;
    
    lo_message msg;

    msg = lo_message_new();
    lo_message_add(msg, "si", "setPlacerType", getPlacerType());
    ret.push_back(msg);
    
    msg = lo_message_new();
    v2 = getRadialRange();
    lo_message_add(msg, "sff", "setRadialRange", v2.x(), v2.y());
    ret.push_back(msg);

    msg = lo_message_new();
    v2 = getRadialPhiRange();
    lo_message_add(msg, "sff", "setRadialiPhiRange", v2.x(), v2.y());
    ret.push_back(msg);
    
    msg = lo_message_new();
    v2 = getCubicXRange();
    lo_message_add(msg, "sff", "setCubicXRange", v2.x(), v2.y());
    ret.push_back(msg);
    
    msg = lo_message_new();
    v2 = getCubicYRange();
    lo_message_add(msg, "sff", "setCubicYRange", v2.x(), v2.y());
    ret.push_back(msg);
    
    msg = lo_message_new();
    v2 = getCubicZRange();
    lo_message_add(msg, "sff", "setCubicZRange", v2.x(), v2.y());
    ret.push_back(msg);
    
    msg = lo_message_new();
    lo_message_add(msg, "s", "removeLinearPlacerVertex");
    ret.push_back(msg);
    for (int i=0; i<linearPlacer_->numVertices(); i++)
    {
        msg = lo_message_new();
        osg::Vec3 v = linearPlacer_->getVertex(i);
        lo_message_add(msg, "sfff", "addLinearPlacerVertex", v.x(), v.y(), v.z());
        ret.push_back(msg);
    }
   
    msg = lo_message_new();
    lo_message_add(msg, "si", "enableOrbiter", getEnabledOrbiter());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "si", "enableAccelerator", getEnabledAccelerator());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "si", "enableAngularAccelerator", getEnabledAngularAccelerator());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "si", "enableAngularDamping", getEnabledAngularDamping());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "si", "enableDamping", getEnabledDamping());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "si", "enableFluidFriction", getEnabledFluidFriction());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "si", "enableExplosion", getEnabledExplosion());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "si", "enableForce", getEnabledForce());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "si", "enableBouncer", getEnabledBouncer());
    ret.push_back(msg);

    // operator params:

    msg = lo_message_new();
    v3 = opAccel_->getAcceleration();
    lo_message_add(msg, "sfff", "setAccel", v3.x(), v3.y(), v3.z());
    ret.push_back(msg);

    msg = lo_message_new();
    v3 = opAngularAccel_->getAngularAcceleration();
    lo_message_add(msg, "sfff", "setAngularAccel", v3.x(), v3.y(), v3.z());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "sf", "setAngularDamping", opAngularDamping_->getDamping().x());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "sf", "setDamping", opDamping_->getDamping().x());
    ret.push_back(msg);

    // Orbit:
    msg = lo_message_new();
    v3 = opOrbit_->getCenter();
    lo_message_add(msg, "sfff", "setOrbitCenter", v3.x(), v3.y(), v3.z());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "sf", "setOrbitMagnitude", opOrbit_->getMagnitude());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "sf", "setOrbitEpsilon", opOrbit_->getEpsilon());
    ret.push_back(msg);
    
    msg = lo_message_new();
    lo_message_add(msg, "sf", "setOrbitMaxRadius", opOrbit_->getMaxRadius());
    ret.push_back(msg);
    
    // Explosion:
    
    msg = lo_message_new();
    lo_message_add(msg, "si", "setExplosionDebugView", getExplosionDebugView());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "ss", "setExplosionTarget", getExplosionTarget().c_str());
    ret.push_back(msg);
    
    msg = lo_message_new();
    v3 = opExplosion_->getCenter();
    lo_message_add(msg, "sfff", "setExplosionCenter", v3.x(), v3.y(), v3.z());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "sf", "setExplosionMaxRadius", opExplosion_->getRadius());
    ret.push_back(msg);
    
    msg = lo_message_new();
    lo_message_add(msg, "sf", "setExplosionMagnitude", opExplosion_->getMagnitude());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "sf", "setExplosionEpsilon", opExplosion_->getEpsilon());
    ret.push_back(msg);
    
    msg = lo_message_new();
    lo_message_add(msg, "sf", "setExplosionSigma", osg::RadiansToDegrees(opExplosion_->getSigma()));
    ret.push_back(msg);

    // Force:
    msg = lo_message_new();
    v3 = opForce_->getForce();
    lo_message_add(msg, "sfff", "setForce", v3.x(), v3.y(), v3.z());
    ret.push_back(msg);

   // FluidFriction:
    msg = lo_message_new();
    lo_message_add(msg, "sf", "setFluidDensity", opFluidFriction_->getFluidDensity());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "sf", "setFluidViscosity", opFluidFriction_->getFluidViscosity());
    ret.push_back(msg);

    msg = lo_message_new();
    v3 = opFluidFriction_->getWind();
    lo_message_add(msg, "sfff", "setFluidDirection", v3.x(), v3.y(), v3.z());
    ret.push_back(msg);

    // Bouncer:
    /*
    msg = lo_message_new();
    lo_message_add(msg, "s", "removeAllBouncers");
    ret.push_back(msg);
    */
    
    msg = lo_message_new();
    lo_message_add(msg, "s", "removeAllBounceTargets");
    ret.push_back(msg);
    
    for (unsigned int i=0; i<bounceTargets_.size(); i++)
    {
        msg = lo_message_new();
        lo_message_add(msg, "ss", "addBounceTarget", bounceTargets_[i]->getID().c_str());
        ret.push_back(msg);
    }
    /*
    for (unsigned int i=0; i<opBouncer_->getNumDomains(); i++)
    {
        switch (opBouncer_->getDomain(i).type)
        {
            case osgParticle::DomainOperator::Domain::PLANE_DOMAIN:
            {
                msg = lo_message_new();
                osg::Vec3 n = opBouncer_->getDomain(i).plane.getNormal();
                osg::Vec3d p;
                if (!getPlaneLineIntersection(opBouncer_->getDomain(i).plane.asVec4(),osg::Vec3(0,0,0),osg::X_AXIS,p))
                    getPlaneLineIntersection(opBouncer_->getDomain(i).plane.asVec4(),osg::Vec3(0,0,0),osg::Z_AXIS,p);
                lo_message_add(msg, "sfff", "addBouncePlane", n.x(), n.y(), n.z(), p.x(), p.y(), p.z());
                ret.push_back(msg);
                break;
            }
            case osgParticle::DomainOperator::Domain::SPHERE_DOMAIN:
            {
                msg = lo_message_new();
                osg::Vec3 c = opBouncer_->getDomain(i).v1;
                float radius = opBouncer_->getDomain(i).r1;
                lo_message_add(msg, "sfff", "addBounceSphere", c.x(), c.y(), c.z(), radius);
                ret.push_back(msg);
                break;
            }
            case osgParticle::DomainOperator::Domain::BOX_DOMAIN:
            {
                msg = lo_message_new();
                osg::Vec3 min = opBouncer_->getDomain(i).v1;
                osg::Vec3 max = opBouncer_->getDomain(i).v2;
                lo_message_add(msg, "sfff", "addBounceSphere", min.x(), min.y(), min.z(), max.x(), max.y(), max.z());
                ret.push_back(msg);
                break;
            }
            default:
                break;
        }
    }
    */
    
    msg = lo_message_new();
    lo_message_add(msg, "si", "setParticleShape", getParticleShape());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "sf", "setLifeTime", getLifeTime());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "sf", "setRadius", getRadius());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "sf", "setMass", getMass());
    ret.push_back(msg);

    msg = lo_message_new();
    range = particle_.getSizeRange();
    lo_message_add(msg, "sff", "setParticleRange", range.minimum, range.maximum);
    ret.push_back(msg);
    
    msg = lo_message_new();
    range = particle_.getAlphaRange();
    lo_message_add(msg, "sff", "setParticleAlphaRange", range.minimum, range.maximum);
    ret.push_back(msg);
    
    msg = lo_message_new();
    const osgParticle::rangev4 rv4 = particle_.getColorRange();
    lo_message_add(msg, "sffffff", "setParticleColorRange", rv4.minimum.x(), rv4.minimum.y(), rv4.minimum.z(), rv4.maximum.x(), rv4.maximum.y(), rv4.maximum.z());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "si", "setEmissive", getEmissive());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "si", "setLighting", getLighting());
    ret.push_back(msg);
    
    msg = lo_message_new();
    lo_message_add(msg, "si", "setUseShaders", getUseShaders());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "ss", "setImagePath", getImagePath());
    ret.push_back(msg);

    msg = lo_message_new();
    v2 = getFrequencyRange();
    lo_message_add(msg, "sff", "setFrequencyRange", v2.x(), v2.y());
    ret.push_back(msg);

    // shooter:
    msg = lo_message_new();
    v2 = getShooterThetaRange();
    lo_message_add(msg, "sff", "setShooterThetaRange", v2.x(), v2.y());
    ret.push_back(msg);

    msg = lo_message_new();
    v2 = getShooterPhiRange();
    lo_message_add(msg, "sff", "setShooterPhiRange", v2.x(), v2.y());
    ret.push_back(msg);

    msg = lo_message_new();
    v2 = getShooterSpeedRange();
    lo_message_add(msg, "sff", "setShooterSpeedRange", v2.x(), v2.y());    ret.push_back(msg);

    msg = lo_message_new();
    const osgParticle::rangev3 rv3 = shooter_->getInitialRotationalSpeedRange();
    lo_message_add(msg, "sffffff", "setShooterRotationalSpeedRange", rv3.minimum.x(), rv3.minimum.y(), rv3.minimum.z(), rv3.maximum.x(), rv3.maximum.y(), rv3.maximum.z());
    ret.push_back(msg);

    return ret;
}

} // end of namespace spin

