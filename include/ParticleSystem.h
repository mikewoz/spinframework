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

#ifndef ParticleSystem_H_
#define ParticleSystem_H_

#include <osgParticle/Particle>
#include <osgParticle/ParticleSystem>
#include <osgParticle/ParticleSystemUpdater>
#include <osgParticle/ModularEmitter>
#include <osgParticle/RandomRateCounter>
#include <osgParticle/SectorPlacer>
#include <osgParticle/RadialShooter>

#include "GroupNode.h"

namespace spin
{

/**
 * \brief A controller for a particle system
 *
 */
class ParticleSystem : public GroupNode
{

public:

    ParticleSystem(SceneManager *sceneManager, char *initID);
    virtual ~ParticleSystem();
    
    virtual void callbackUpdate(osg::NodeVisitor* nv);
    
    void setLifeTime(float seconds);
    float getLifeTime() { return (float) particle_.getLifeTime(); }

    void setRadius(float radius);
    float getRadius() { return (float) particle_.getRadius(); }

    void setMass(float mass);
    float getMass() { return (float) particle_.getMass(); }
    
    void setEmissive(int emissiveFlag);
    int getEmissive() const { return (int) emissive_; }

    void setLighting(int lightingFlag);
    int getLighting() const { return (int) lighting_; }

    void setImagePath(const char* path);
    const char* getImagePath() const { return imgPath_.c_str(); }
    
    void setFrequencyRange(float min, float max);
    osg::Vec2 getFrequencyRange() const { return freqRange_; }

    void setCircularRange(float min, float max);
    osg::Vec2 getCircularRange();
    
    void setSpeedRange(float min, float max);
    osg::Vec2 getSpeedRange();


    virtual std::vector<lo_message> getState() const;

private:

    bool attachedFlag_;

    osg::ref_ptr<osgParticle::ParticleSystem> system_;
    osg::ref_ptr<osgParticle::ParticleSystemUpdater> updater_;
    osg::ref_ptr<osgParticle::ModularEmitter> emitter_;
    osg::ref_ptr<osgParticle::RandomRateCounter> counter_;
    osg::ref_ptr<osgParticle::SectorPlacer> placer_;
    osg::ref_ptr<osgParticle::RadialShooter> shooter_;
    
    osgParticle::Particle particle_;
    
    float lifetime_;
    float radius_;
    float mass_;
    
    osg::Vec2 freqRange_;
    osg::Vec2 circularRange_;
    osg::Vec2 angularRange_;
    osg::Vec2 speedRange_;
    osg::Vec2 alphaRange_;
    osg::Vec2 sizeRange_;
    osg::Vec4 colorMin_;
    osg::Vec4 colorMax_;
    
    std::string imgPath_;
    bool emissive_;
    bool lighting_;
};

} // end of namespace spin

#endif
