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

#include <osgParticle/Particle>
#include <osgParticle/ParticleSystem>
#include <osgParticle/ParticleSystemUpdater>
#include <osgParticle/ModularEmitter>
#include <osgParticle/ModularProgram>
#include <osgParticle/RandomRateCounter>
#include <osgParticle/SectorPlacer>
#include <osgParticle/RadialShooter>
#include <osgParticle/AccelOperator>
#include <osgParticle/FluidFrictionOperator>

#include "ParticleSystem.h"
#include "SceneManager.h"
#include "spinApp.h"
#include "spinBaseContext.h"
//#include "osgUtil.h"

using namespace std;

namespace spin
{

// *****************************************************************************
// constructor:
ParticleSystem::ParticleSystem (SceneManager *sceneManager, char *initID) : GroupNode(sceneManager, initID)
{
	this->setName(string(id->s_name) + ".ParticleSystem");
	nodeType = "ParticleSystem";

    imgPath_.clear();
    attachedFlag_ = false;
    emissive_ = true;
    lighting_ = false;
    rate_ = osg::Vec2(10.0, 30.0); // random rate between 10-30 particles/second
    

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
    
    
    // Ok, get a pointer to the emitter's Counter object. We could also
    // create a new RandomRateCounter object and assign it to the emitter,
    // but since the default counter is already a RandomRateCounter, we
    // just get a pointer to it and change a value:
    osgParticle::RandomRateCounter *rrc =
        static_cast<osgParticle::RandomRateCounter *>(emitter_->getCounter());

    // Now set the rate range to a better value. The actual rate at each frame
    // will be chosen randomly within that range.
    rrc->setRateRange(rate_.x(), rate_.y());
    
    // The emitter is done! Let's attach it:
    this->getAttachmentNode()->addChild(emitter_);
    
    
    // We don't add any particle modifier 
    // here (see ModularProgram and Operator classes), so all we still need is
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
void ParticleSystem::callbackUpdate()
{
    GroupNode::callbackUpdate();
    
    if (!attachedFlag_)
    {
        this->getAttachmentNode()->addChild(updater_);
        attachedFlag_ = true;
    }
}
    
// *****************************************************************************
void ParticleSystem::setRate (float min, float max)
{
    rate_ = osg::Vec2(min,max);
    
    // make sure rate is above zero and that max is bigger (or equal) to min:
    if (rate_.x() < 0) rate_.x() = 0;
    if (rate_.y() < rate_.x()) rate_.y() = rate_.x();

    // Get a pointer to the emitter's Counter object:
    osgParticle::RandomRateCounter *rrc =
        static_cast<osgParticle::RandomRateCounter *>(emitter_->getCounter());

    // Set the min and max rate (number of particles per second). The actual
    // rate at each frame will be chosen randomly within that range.
    rrc->setRateRange(rate_.x(), rate_.y());

	BROADCAST(this, "sff", "setRate", rate_.x(), rate_.y());
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
    osg::Vec2 v = getRate();
    lo_message_add(msg, "sff", "setRate", v.x(), v.y());
    ret.push_back(msg);

    return ret;
}

} // end of namespace spin

