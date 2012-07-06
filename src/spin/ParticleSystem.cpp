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


#include <osgParticle/AccelOperator>
#include <osgParticle/FluidFrictionOperator>

#include "ParticleSystem.h"
#include "SceneManager.h"
#include "spinApp.h"
#include "spinBaseContext.h"

namespace spin
{

// *****************************************************************************
// constructor:
ParticleSystem::ParticleSystem (SceneManager *sceneManager, const char* initID) : GroupNode(sceneManager, initID)
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
    circularRange_ = osg::Vec2(0.0, 2.0); // circle of 0-2m
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
    emitter_->setName("particle emitter");
    
    
    // the first thing you *MUST* do after creating an emitter is to set the
    // destination particle system, otherwise it won't know where to create
    // new particles.

    emitter_->setParticleSystem(system_);
    
    
    // set the counter:
    counter_ = new osgParticle::RandomRateCounter;
    counter_->setRateRange(freqRange_.x(), freqRange_.y());
    emitter_->setCounter(counter_);
    

    // set the placer:
    placer_ = new osgParticle::SectorPlacer;
    placer_->setCenter(0,0,0);
    placer_->setRadiusRange(0, 0);
    placer_->setPhiRange(0, 2 * osg::PI);
    emitter_->setPlacer(placer_);


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
    
    osgParticle::ModularProgram *program = new osgParticle::ModularProgram;
    program->setParticleSystem(system_);

    // create an operator that simulates the gravity acceleration.
    osgParticle::AccelOperator *op1 = new osgParticle::AccelOperator;
    op1->setToGravity();
    program->addOperator(op1);

    // now create a custom operator, we have defined it above
    /*
    VortexOperator *op2 = new VortexOperator;
    op2->setCenter(osg::Vec3(8, 0, 0));
    program->addOperator(op2);
    */

    // let's add a fluid operator to simulate air friction.
    osgParticle::FluidFrictionOperator *op3 = new osgParticle::FluidFrictionOperator;
    op3->setFluidToAir();
    program->addOperator(op3);

    // add the program to the scene graph
    this->getAttachmentNode()->addChild(program);
    
    
    // all we still need is
    // to create a Geode and add the particle system to it, so it can be
    // displayed.

    osg::Geode *geode = new osg::Geode;
    geode->setName("particle system");
    geode->addDrawable(system_);

    // add the geode to the scene graph
    this->getAttachmentNode()->addChild(geode);
    
    
    updater_ = new osgParticle::ParticleSystemUpdater;
    updater_->setName("particle updater");
    updater_->addParticleSystem(system_);
    //this->getAttachmentNode()->addChild(updater_);

}

// *****************************************************************************
// destructor
ParticleSystem::~ParticleSystem()
{

}


// *****************************************************************************
void ParticleSystem::callbackUpdate(osg::NodeVisitor* nv)
{
    GroupNode::callbackUpdate(nv);
    
    if (!attachedFlag_)
    {
        this->getAttachmentNode()->addChild(updater_);
        attachedFlag_ = true;
    }
}
    
// *****************************************************************************

void ParticleSystem::setLifeTime(float seconds)
{
    particle_.setLifeTime(seconds);
    BROADCAST(this, "sf", "setLifeTime", getLifeTime());
}

void ParticleSystem::setRadius(float radius)
{
    particle_.setRadius(radius);
    BROADCAST(this, "sf", "setRadius", getRadius());
}

void ParticleSystem::setMass(float mass)
{
    particle_.setMass(mass);
    BROADCAST(this, "sf", "setMass", getMass());
}

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

void ParticleSystem::setCircularRange(float min, float max)
{
    placer_->setRadiusRange(min, max);
    BROADCAST(this, "sff", "setFrequencyRange", min, max);
    //BROADCAST(this, "sff", "setCircularRange", range.minimum, range.maximum);
}
osg::Vec2 ParticleSystem::getCircularRange()
{
    const osgParticle::rangef range = placer_->getRadiusRange();
    return osg::Vec2(range.minimum, range.maximum);
}


void ParticleSystem::setSpeedRange(float min, float max)
{
    shooter_->setInitialSpeedRange(min, max);
    BROADCAST(this, "sff", "setSpeedRange", min, max);
}
osg::Vec2 ParticleSystem::getSpeedRange()
{
    const osgParticle::rangef range = shooter_->getInitialSpeedRange();
    return osg::Vec2(range.minimum, range.maximum);
}


void ParticleSystem::setImagePath (const char* path)
{
    if (imgPath_ != path)
    {
        imgPath_ = path;
        system_->setDefaultAttributes(imgPath_, emissive_, lighting_);
    
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

// *****************************************************************************

std::vector<lo_message> ParticleSystem::getState () const
{
    // inherit state from base class
	std::vector<lo_message> ret = GroupNode::getState();

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

