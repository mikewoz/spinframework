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

#include "CollisionShape.h"
#include "spinApp.h"
#include "spinBaseContext.h"
#include "SceneManager.h"
#include "osgUtil.h"
#include "bulletUtil.h"


using namespace std;

//extern pthread_mutex_t sceneMutex;
//extern ContactAddedCallback gContactAddedCallback;


namespace spin
{



// -----------------------------------------------------------------------------
// constructor:
CollisionShape::CollisionShape (SceneManager *sceneManager, char *initID) : ShapeNode(sceneManager, initID)
{
	this->setName(string(id->s_name) + ".CollisionShape");
	nodeType = "CollisionShape";

    mass_ = 1.0;
    isDynamic_ = false;
    
    //gContactAddedCallback = btCollisionCallback2;


    collisionObj_ = new btBoxShape(btVector3(0.5,0.5,0.5));
    
    currentTransform_.setIdentity();
    
    btVector3 localInertia(0,0,0);
    if (mass_!=0.f) collisionObj_->calculateLocalInertia(mass_, localInertia);


    // using motionstate is recommended, it provides interpolation capabilities,
    // and only synchronizes 'active' objects
    btDefaultMotionState* myMotionState = new btDefaultMotionState(currentTransform_);
    
    body_ = new btRigidBody(mass_, myMotionState, collisionObj_, localInertia);


    body_->setUserPointer(this);
    
    /*
    body_->setCollisionFlags(body_->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
    body_->setCcdMotionThreshold(0.5);
    body_->setCcdSweptSphereRadius(0.2 * 0.5);
    */
    
    body_->setCollisionFlags( body_->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
    body_->setActivationState( DISABLE_DEACTIVATION );

  
  
    sceneManager->dynamicsWorld_->addCollisionObject(body_);

    this->setInteractionMode(GroupNode::DRAG);
}

// destructor
CollisionShape::~CollisionShape()
{
}

// -----------------------------------------------------------------------------
void CollisionShape::callbackUpdate()
{
    ShapeNode::callbackUpdate();
    
    
    // If this is the server, we update the position of the node based on info
    // from the dynamics engine... but only if the dynamics are currently on and
    // the user is not currently manipulating the object.
    
    
    if (isDynamic_ && (mass_!=0))
    {
        btScalar m[16];

        btDefaultMotionState* myMotionState = (btDefaultMotionState*) body_->getMotionState();
        myMotionState->m_graphicsWorldTrans.getOpenGLMatrix(m);

        osg::Matrixf mat(m);
        
        this->setTranslation(mat.getTrans().x(),mat.getTrans().y(),mat.getTrans().z());
    }
}

void CollisionShape::debug()
{
    ShapeNode::debug();
    
    std::cout << "   ---------" << std::endl;
    
    std::cout << "   mass = " << mass_ << std::endl;
    
    
    btScalar m[16];
    btDefaultMotionState* myMotionState = (btDefaultMotionState*) body_->getMotionState();
    myMotionState->m_graphicsWorldTrans.getOpenGLMatrix(m);
    osg::Matrixf mat(m);
    
    std::cout << "   phys trans: " << stringify(mat.getTrans()) << std::endl;
    std::cout << "   phys orien: " << stringify(mat.getRotate()) << std::endl;
    std::cout << "   phys scale: " << stringify(mat.getScale()) << std::endl;

    btTransform t = body_->getWorldTransform();
    
    std::cout << "   wrld trans: " << stringify(asOsgVec3(t.getOrigin())) << std::endl;
    std::cout << "   wrld orien: " << stringify(asOsgQuat(t.getRotation())) << std::endl;

    
}

// -----------------------------------------------------------------------------


void CollisionShape::setMass (float mass)
{
    mass_ = mass;
    
    btVector3 localInertia(0, 0, 0);
    
    // rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass_ != 0.f);
    if (isDynamic)
        collisionObj_->calculateLocalInertia(mass_, localInertia);
    
    collisionObj_->calculateLocalInertia(mass_, localInertia);
    body_->setMassProps(mass_, localInertia);

	BROADCAST(this, "sf", "setMass", getMass());
}

void CollisionShape::setDynamic(int isDynamic)
{
    isDynamic_ = isDynamic;
    
	BROADCAST(this, "si", "setDynamic", getDynamic());
}

void CollisionShape::setTranslation (float x, float y, float z)
{
    ShapeNode::setTranslation(x,y,z);
    
    // TODO: use global matrix
    
    btVector3 pos(mainTransform->getPosition().x(), mainTransform->getPosition().y(), mainTransform->getPosition().z());
        
    currentTransform_.setOrigin(pos);
    
    //body_->setWorldTransform(currentTransform_);
    body_->getMotionState()->setWorldTransform(currentTransform_);
}

void CollisionShape::setOrientation(float pitch, float roll, float yaw)
{
    ShapeNode::setOrientation(pitch, roll, yaw);
    
    // TODO: use global matrix
    
    btQuaternion quat(mainTransform->getAttitude().x(), mainTransform->getAttitude().y(), mainTransform->getAttitude().z(), mainTransform->getAttitude().w());
    currentTransform_.setRotation(quat);
    
    //body_->setWorldTransform(currentTransform_);
    body_->getMotionState()->setWorldTransform(currentTransform_);
}

void CollisionShape::setScale(float x, float y, float z)
{
    ShapeNode::setScale(x,y,z);
    
    // TODO: use global matrix
    
    //body_->setWorldTransform(currentTransform_);
    collisionObj_->setLocalScaling(asBtVector3(mainTransform->getScale()));
}

// -----------------------------------------------------------------------------
void CollisionShape::drawShape()
{
    ShapeNode::drawShape();
    
    // update the collision object's shape:

}

// -----------------------------------------------------------------------------
std::vector<lo_message> CollisionShape::getState () const
{
	// inherit state from base class
	std::vector<lo_message> ret = ShapeNode::getState();

	lo_message msg;
    
	msg = lo_message_new();
	lo_message_add(msg, "sf", "setMass", getMass());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "si", "setDynamic", getDynamic());
	ret.push_back(msg);

	
	return ret;
}

} // end of namespace spin

