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

#ifndef __CollisionShape_H
#define __CollisionShape_H

#include "ShapeNode.h"
#include <btBulletDynamicsCommon.h>
#include <btBulletCollisionCommon.h>


namespace spin
{

/**
 * \brief Provides an interface to bullet physics modelling
 */
class CollisionShape : public ShapeNode
{
public:

    CollisionShape(SceneManager *sceneManager, const char* initID);
    virtual ~CollisionShape();
    
    virtual void callbackUpdate(osg::NodeVisitor* nv);
    virtual void debug();
    
    /**
     * Set the mass of the object. A value of 0 makes the object static
     * (unmovable) by the dynamics engine. Only direct transformation through
     * GroupNode functions will have any effect.
     */
    void setMass(float mass);
    float getMass() const { return (float)mass_; };

    /**
     * Enable external dynamic forces (eg, gravity). Note: object must have
     * non-zero mass for this to work.
     */
    void setDynamic(int isDynamic);
    int getDynamic() const { return (int)isDynamic_; };

    bool checkCollisions(btTransform tranform);
    virtual void setTranslation (float x, float y, float z);
    virtual void setOrientationQuat(float x, float y, float z, float w);
    virtual void setOrientation (float pitch, float roll, float yaw);
    virtual void setScale (float x, float y, float z);

    virtual void setManipulatorMatrix
        (float a00, float a01, float a02, float a03,
         float a10, float a11, float a12, float a13,
         float a20, float a21, float a22, float a23,
         float a30, float a31, float a32, float a33);


    virtual std::vector<lo_message> getState() const;
    
    osg::Vec3 collisionOffset_;

protected:

    virtual void drawShape();

    btRigidBody* body_;
    btCollisionShape *collisionObj_;
    
    btTransform currentTransform_;

private:

    bool isDynamic_;
    btScalar mass_;
    
    osg::Timer_t lastTick_;

};

} // end of namespace spin

#endif
