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

#include <osg/Version>
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

#include "ConstraintsNode.h"

namespace spin
{

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
/*
static void computeNewBasis( const osg::Vec3& u, const osg::Vec3& v, osg::Vec3& s1, osg::Vec3& s2 )
{
    // Copied from David McAllister's Particle System API (http://www.particlesystems.org), pDomain.h
    osg::Vec3 w = u ^ v;
    float det = w.z()*u.x()*v.y() - w.z()*u.y()*v.x() - u.z()*w.x()*v.y() -
    u.x()*v.z()*w.y() + v.z()*w.x()*u.y() + u.z()*v.x()*w.y();
    det = 1.0f / det;
    
    s1.set( v.y()*w.z() - v.z()*w.y(), v.z()*w.x() - v.x()*w.z(), v.x()*w.y() - v.y()*w.x() );
    s1 = s1 * det;
    s2.set( u.y()*w.z() - u.z()*w.y(), u.z()*w.x() - u.x()*w.z(), u.x()*w.y() - u.y()*w.x() );
    s2 = s2 * (-det);
}
*/  

    
/**
 * \brief A controller for a particle system
 *
 * See http://www.openscenegraph.org/projects/osg/wiki/Support/Tutorials/ParticleEffects
*/
class ParticleSystem : public ConstraintsNode
{

public:

    ParticleSystem(SceneManager *sceneManager, const char* initID);
    virtual ~ParticleSystem();
    
    enum PlacerType
    {
        RADIAL,     /*!< Places particles randomly within a radial disc. A range
                    (minRadius,maxRadius) can be specified, as well as a range
                    for the central angle (phi). */
        CUBIC,      /*!< Places particles randomly within a cubic box. You can
                    specify the x-range, y-range, and z-range. */
        LINEAR      /*!< Places particles randomly along a series of line
                    segments. */
    };
    
    
    virtual void callbackUpdate(osg::NodeVisitor* nv);
    void updateNodePath(bool updateChildren);
    
    virtual void debug();
    
    
    void setPlacerType(int type);
    int  getPlacerType() const;
    
    void setRadialRange(float min, float max);
    osg::Vec2 getRadialRange() const;
    
    void setRadialPhiRange(float min, float max);
    osg::Vec2 getRadialPhiRange() const;
    
    void setCubicXRange(float min, float max);
    osg::Vec2 getCubicXRange() const;
    void setCubicYRange(float min, float max);
    osg::Vec2 getCubicYRange() const;
    void setCubicZRange(float min, float max);
    osg::Vec2 getCubicZRange() const;
    
    void addLinearPlacerVertex(float x, float y, float z);
    void removeLinearPlacerVertices();

    
    /**
     * The orbiter forces particles in the orbit around a point.
     */
    void enableOrbiter(int b);
    int getEnabledOrbiter() const;

    /**
     * Applies a constant acceleration to the particles (eg, gravity).
     */
    void enableAccelerator(int b);
    int getEnabledAccelerator() const;
    
    /**
     * Applies a constant angular acceleration to the particles.
     */
    void enableAngularAccelerator(int b);
    int getEnabledAngularAccelerator() const;

    /**
     * Applies damping constant to particle's angular velocity.
     */
    void enableAngularDamping(int b);
    int getEnabledAngularDamping() const;

    /**
     * Applies damping constant to particle's velocity.
     */
    void enableDamping(int b);
    int getEnabledDamping() const;

    /**
     * Simulates the friction of a fluid.
     *
     * By using this operator you can let the particles move in a fluid of a
     * given density and viscosity. There are two functions to quickly setup the
     * parameters for pure water and air. You can decide whether to compute the
     * forces using the particle's physical radius or another value, by calling
     * the setOverrideRadius() method.
     */
    void enableFluidFriction(int b);
    int getEnabledFluidFriction() const;
   
    /**
     * Exerts force on each particle away from the explosion center.
     */
    void enableExplosion(int b);
    int getEnabledExplosion() const;
   
    /**
     * Applies a constant force to the particles. Remember that if the mass of
     * particles is expressed in kg and the lengths are expressed in meters,
     * then the force should be expressed in Newtons.
     */
    void enableForce(int b);
    int getEnabledForce() const;
   
    /**
     * Can affect the particle's velocity to make it rebound. 
     */
    void enableBouncer(int b);
    int getEnabledBouncer() const;

    // OPERATOR PARAMS:
    void setAccel(float x, float y, float z);
    void setAngularAccel(float x, float y, float z);
    void setAngularDamping(float d);
    void setDamping(float d);
    
    void setOrbitCenter(float x, float y, float z);
    void setOrbitMagnitude(float mag);
    void setOrbitEpsilon(float eps);
    void setOrbitMaxRadius(float max);
    
    void setExplosionTarget(const char* targetID);
    std::string getExplosionTarget() const { if (opExplosionTarget_.valid()) return opExplosionTarget_->getID(); else return "NULL"; }
    void setExplosionDebugView(int b);
    int  getExplosionDebugView() const { return (int)this->containsNode(opExplosionDebugView_.get()); }
    /// sets the center of the explosion (Note: an explosion target overrides this)
    void setExplosionCenter(float x, float y, float z);
    /// radius defines how far away the shockwave peaks
    void setExplosionRadius(float r);
    /// magnitude defines the amount of force exerted on the particles. Try 100+
    void setExplosionMagnitude(float mag);
    /// epsilon sets the distance from the center
    void setExplosionEpsilon(float eps);
    /// sigma (in degrees) determines the broadness of the explosion shockwave
    void setExplosionSigma(float s);

    void setForce(float x, float y, float z);
    
    void setFluidDensity(float d);
    void setFluidViscosity(float v);
    void setFluidDirection(float x, float y, float z);
    
    
    void addBounceTarget(const char* nodeID);
    void removeBounceTarget(const char* nodeID);
    void removeAllBounceTargets();

    void addBouncePlane(float normalX, float normalY, float normalZ, float x, float y, float z);
    void addBounceBox(float minX, float minY, float minZ, float maxX, float maxY, float maxZ);
    void addBounceSphere(float x, float y, float z, float radius);
    void removeAllBouncers();
    void setBounceFriction(float f);
    void setBounceResilience(float r);
    void setBounceCutoff(float v);
    

    // override some methods :
    virtual void setTranslation (float x, float y, float z);
    virtual void setOrientation (float p, float r, float y);
    virtual void setOrientationQuat (float x, float y, float z, float w);
    virtual osg::Vec3 getTranslation() const { return radialPlacer_->getCenter(); }
    virtual void updateStateSet();

    // particle properties:
    void setParticleShape(int shp);
    int getParticleShape() const { return (int) particle_.getShape(); }

    void setLifeTime(float seconds);
    float getLifeTime() const { return (float) particle_.getLifeTime(); }

    void setRadius(float radius);
    float getRadius() const { return (float) particle_.getRadius(); }

    void setMass(float mass);
    float getMass() const { return (float) particle_.getMass(); }
    
    void setParticleSizeRange(float min, float max);
    void setParticleAlphaRange(float min, float max);
    void setParticleColorRange(float minR, float minG, float minB, float maxR, float maxG, float maxB);
    
    
    void setEmissive(int emissiveFlag);
    int getEmissive() const { return (int) emissive_; }

    void setLighting(int lightingFlag);
    int getLighting() const { return (int) lighting_; }

    void setUseShaders(int shaderFlag);
    int getUseShaders() const { return (int) system_->getUseShaders(); }
    
    void setImagePath(const char* path);
    const char* getImagePath() const { return imgPath_.c_str(); }
    
    void setImage (const char* path);
     
    void setFrequencyRange(float min, float max);
    osg::Vec2 getFrequencyRange() const { return freqRange_; }

    void setShooterThetaRange(float min, float max);
    osg::Vec2 getShooterThetaRange() const;
    void setShooterPhiRange(float min, float max);
    osg::Vec2 getShooterPhiRange() const;
    void setShooterSpeedRange(float min, float max);
    osg::Vec2 getShooterSpeedRange() const;
    void setShootertRotationalSpeedRange(float minX, float minY, float minZ, float maxX, float maxY, float maxZ);


    virtual std::vector<lo_message> getState() const;

private:

    void removeOperator(osgParticle::Operator *op);

    bool attachedFlag_;

    //osg::ref_ptr<osgParticle::ConnectedParticleSystem> system_;
    osg::ref_ptr<osgParticle::ParticleSystem> system_;
    osg::ref_ptr<osgParticle::ParticleSystemUpdater> updater_;
    osg::ref_ptr<osgParticle::ModularEmitter> emitter_;
    osg::ref_ptr<osgParticle::RandomRateCounter> counter_;
    osg::ref_ptr<osgParticle::SectorPlacer> radialPlacer_;
    osg::ref_ptr<osgParticle::BoxPlacer> cubicPlacer_;
    osg::ref_ptr<osgParticle::MultiSegmentPlacer> linearPlacer_;
    osg::ref_ptr<osgParticle::RadialShooter> shooter_;
    
    // programs and their operators:
    osg::ref_ptr<osgParticle::ModularProgram> program_;

    osg::ref_ptr<osgParticle::AccelOperator> opAccel_;
    osg::ref_ptr<osgParticle::AngularAccelOperator> opAngularAccel_;
    osg::ref_ptr<osgParticle::FluidFrictionOperator> opFluidFriction_;
    osg::ref_ptr<osgParticle::ForceOperator> opForce_;

#ifdef OSG_MIN_VERSION_REQUIRED
#if OSG_MIN_VERSION_REQUIRED(3,0,0)
    osg::ref_ptr<osgParticle::OrbitOperator> opOrbit_;
    osg::ref_ptr<osgParticle::AngularDampingOperator> opAngularDamping_;
    osg::ref_ptr<osgParticle::DampingOperator> opDamping_;
    osg::ref_ptr<osgParticle::ExplosionOperator> opExplosion_;
    osg::ref_ptr<BouncerOperator> opBouncer_;
#else
    osg::ref_ptr<NullOperator> opOrbit_;
    osg::ref_ptr<NullOperator> opAngularDamping_;
    osg::ref_ptr<NullOperator> opDamping_;
    osg::ref_ptr<NullOperator> opExplosion_;
    osg::ref_ptr<NullOperator> opBouncer_;
#endif
#endif
    
    osg::ref_ptr<osg::PositionAttitudeTransform> opExplosionDebugView_;
    
    osg::observer_ptr<GroupNode> opExplosionTarget_;
    std::vector< osg::observer_ptr<ReferencedNode> > bounceTargets_;
    
    osg::ref_ptr<osg::PositionAttitudeTransform> attachPAT;
    
    osgParticle::Particle particle_;
    
    float lifetime_;
    float radius_;
    float mass_;
    
    osg::Vec2 freqRange_;
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
