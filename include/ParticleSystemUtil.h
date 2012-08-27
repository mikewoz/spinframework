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

#ifndef ParticleSystemUtil_H_
#define ParticleSystemUtil_H_

#include <osg/Version>
#include <osgParticle/range>
#include <osgParticle/Particle>
#include <osgParticle/ParticleSystem>
#include <osgParticle/ConnectedParticleSystem>
#include <osgParticle/ParticleSystemUpdater>
#include <osgParticle/ModularEmitter>
#include <osgParticle/RandomRateCounter>
#include <osgParticle/SectorPlacer>
#include <osgParticle/BoxPlacer>
#include <osgParticle/MultiSegmentPlacer>
#include <osgParticle/RadialShooter>

#include <osgParticle/AccelOperator>
#include <osgParticle/AngularAccelOperator>
#include <osgParticle/FluidFrictionOperator>
#include <osgParticle/ForceOperator>

#ifdef OSG_MIN_VERSION_REQUIRED
#if OSG_MIN_VERSION_REQUIRED(3,0,0)
#include <osgParticle/OrbitOperator>
#include <osgParticle/AngularDampingOperator>
#include <osgParticle/DampingOperator>
#include <osgParticle/ExplosionOperator>
#include <osgParticle/BounceOperator>
#endif
#endif


namespace spin
{

    
class SwitchableParticleSystem : public osgParticle::ConnectedParticleSystem
{
public:
    SwitchableParticleSystem() : osgParticle::ConnectedParticleSystem(), connected_(false)
    {
        _maxNumberOfParticlesToSkip = 0;
    }
    SwitchableParticleSystem(const SwitchableParticleSystem& copy, const osg::CopyOp& copyop = osg::CopyOp::SHALLOW_COPY) {}
    
    META_Object(spin, SwitchableParticleSystem);
    
    // override some functions to allow switching between connected and non-connected mode:
    virtual osgParticle::Particle* createParticle(const osgParticle::Particle* ptemplate)
    {
        if (connected_) return osgParticle::ConnectedParticleSystem::createParticle(ptemplate);
        else return osgParticle::ParticleSystem::createParticle(ptemplate);
    }
    virtual void reuseParticle(int i)
    {
        if (connected_) osgParticle::ConnectedParticleSystem::reuseParticle(i);
        else osgParticle::ParticleSystem::reuseParticle(i);
    }
    virtual void drawImplementation(osg::RenderInfo& renderInfo) const
    {
        if (connected_) osgParticle::ConnectedParticleSystem::drawImplementation(renderInfo);
        else return osgParticle::ParticleSystem::drawImplementation(renderInfo);        
    }
    
    void setConnected(bool b)
    {
        connected_ = b;
        if (b)
        {
            _startParticle = osgParticle::Particle::INVALID_INDEX;
            _lastParticleCreated = osgParticle::Particle::INVALID_INDEX;
        }
    }
    bool getConnected() const { return connected_; }
    
protected:
    virtual ~SwitchableParticleSystem() {}
    
    SwitchableParticleSystem& operator=(const SwitchableParticleSystem&) { return *this; }
    
    bool connected_;
};
    

    
/**
 * A fake operator class in case the user doesn't have the most recent version
 * of OSG (and is thus missing an operator class).
 */
class NullOperator : public osgParticle::Operator
{
    NullOperator() : osgParticle::Operator() {}
    inline void operate(osgParticle::Particle* P, double dt) { (void(P)); (void(dt)); }
};

/**
 * We override OSG's BounceOperator class so that we can get and dynamically
 * update domains:
 */
class BouncerOperator : public osgParticle::BounceOperator
{
public:
    BouncerOperator() : osgParticle::BounceOperator() {}
    osgParticle::DomainOperator::Domain* getDomainPointer( unsigned int i )
    { return &_domains[i]; }
    void updatePlane(osgParticle::DomainOperator::Domain* d)
    { computeNewBasis(d->v2, d->v3, d->s1, d->s2); }
    
};

/**
 * Applies an attractive force towards a specific point.
 */
    
class AttractOperator : public osgParticle::Operator
{
public:
    AttractOperator() : osgParticle::Operator(), _magnitude(1.0f), _ratio(0.5), _killSink(true)
    {}
    
    AttractOperator( const AttractOperator& copy, const osg::CopyOp& copyop = osg::CopyOp::SHALLOW_COPY )
    :   osgParticle::Operator(copy, copyop), _center(copy._center), _magnitude(copy._magnitude), _ratio(copy._ratio), _killSink(copy._killSink)
    {}
    
    META_Object( spin, AttractOperator );
    
    /// Set the center of the attractive force
    void setCenter( const osg::Vec3& c ) { _center = c; }
    /// Get the center of the attractive force
    const osg::Vec3& getCenter() const { return _center; }
    
    /// Set the acceleration scale
    void setMagnitude( float mag ) { _magnitude = mag; }
    /// Get the acceleration scale
    float getMagnitude() const { return _magnitude; }

    /// Set the attraction ratio (CURRENTLY UNUSED)
    void setRatio( float r ) { _ratio = r; if (_ratio<0.0f) _ratio=0.0f; if (_ratio>1.0f) _ratio=1.0f; }
    /// Get the attraction ratio
    float getRatio() const { return _ratio; }
    
    /// Set whether the attractor kills the particles once they arrive
    void setKillSink( bool kill ) { _killSink = kill; }
    /// Get whether the attractor kills the particles once they arrive
    bool getKillSink() const { return _killSink; }
    
    /// Apply attraction to a particle. Do not call this method manually.
    inline void operate( osgParticle::Particle* P, double dt );
    
    /// Perform some initializations. Do not call this method manually.
    inline void beginOperate( osgParticle::Program* prg );
    
protected:
    virtual ~AttractOperator() {}
    AttractOperator& operator=( const AttractOperator& ) { return *this; }
    
    osg::Vec3 _center;
    osg::Vec3 _xf_center;
    float _magnitude;
    float _ratio;
    bool _killSink;
};


inline void AttractOperator::operate( osgParticle::Particle* P, double dt )
{
    osg::Vec3 dir = _xf_center - P->getPosition();
    if (dir.length()>0.1 )
    {
        // similar to orbit (but without epsilon):
        //P->addVelocity( dir * _magnitude * dt * (1-_ratio) );
        
        // close, but changes absolute position when we move center:
        //P->addVelocity( dir * _magnitude * dt * (1-_ratio) );
        //P->setPosition(P->getPosition() + (dir * _magnitude * dt * _ratio));

        // make rotation from current direction to target direction:
        osg::Matrix mat;
        osg::Vec3 current = P->getVelocity();
        
        current.normalize();
        dir.normalize();
        mat.makeRotate(current, dir);
        
        float scalar_factor = 0.01;
        P->transformPositionVelocity(osg::Matrix::identity(), mat, 1/(1+(_magnitude*_magnitude*0.001)) );
        //P->transformPositionVelocity(osg::Matrix::identity(), mat, _ratio);
    }
    else
    {
        if (_killSink)
        {
            P->kill();
        }
        else
        {
            P->setPosition(_xf_center);
            P->setVelocity(osg::Vec3(0,0,0));
        }
    }
}

inline void AttractOperator::beginOperate( osgParticle::Program* prg )
{
    if ( prg->getReferenceFrame()==osgParticle::ModularProgram::RELATIVE_RF )
    {
        _xf_center = prg->transformLocalToWorld(_center);
    }
    else
    {
        _xf_center = _center;
    }
}

    
    
/**
 * Applies an attractive force towards a specific point.
 */

class OscillatorOperator : public osgParticle::Operator
{
public:
    OscillatorOperator() : osgParticle::Operator(), _amplitude(1.0f), _frequency(1.0f), _angleLock(false)
    {}
    
    OscillatorOperator( const OscillatorOperator& copy, const osg::CopyOp& copyop = osg::CopyOp::SHALLOW_COPY )
    :   osgParticle::Operator(copy, copyop), _amplitude(copy._amplitude), _frequency(copy._frequency), _angleLock(copy._angleLock)
    {}
    
    META_Object( spin, OscillatorOperator );
    
    /// Set the amplitude scalar
    void setAmplitude( float amp ) { _amplitude = amp; }
    /// Get the amplitude
    float getAmplitude() const { return _amplitude; }
    
    /// Set frequency
    void setFrequency( float f ) { _frequency = f; if (_frequency<0.0f) _frequency=0.0f; }
    /// Get the frequency
    float getFrequency() const { return _frequency; }
    
    /// Set whether the particle's angle is locked to the oscillation
    void setAngleLock( bool lock ) { _angleLock = lock; }
    /// Get whether the particle's angle is locked to the oscillation
    bool getAngleLock() const { return _angleLock; }
    
    /// Apply attraction to a particle. Do not call this method manually.
    inline void operate( osgParticle::Particle* P, double dt );
    
protected:
    virtual ~OscillatorOperator() {}
    OscillatorOperator& operator=( const OscillatorOperator& ) { return *this; }
    
    float _amplitude;
    float _frequency;
    bool _angleLock;
    
};


inline void OscillatorOperator::operate( osgParticle::Particle* P, double dt )
{
    // try to get a vector orthogonal to the current velocity:
    osg::Vec3 norm = P->getVelocity() ^ osg::Y_AXIS;
    // check that norm is valid (ie, not parallel)
    if (norm.length()<0.0001) norm = P->getVelocity() ^ osg::X_AXIS;
    
    norm.normalize();
    
    osg::Vec3 offset = norm * cos(P->getAge() * _frequency) * _amplitude/100;
    
    P->setPosition(P->getPosition() + offset);
    
    if (_angleLock)
    {
        //TODO
        P->setAngle(norm);
    }
}

    
} // end namespace

#endif