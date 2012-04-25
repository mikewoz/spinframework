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

extern float lastBtHitDepth;

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
    
    //Collision objects with a callback still have collision response with dynamic rigid bodies. In order to use collision objects as trigger, you have to disable the collision response.
    //mBody->setCollisionFlags(mBody->getCollisionFlags() |btCollisionObject::CF_NO_CONTACT_RESPONSE));
    
    body_->setCollisionFlags( body_->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK | btCollisionObject::CF_NO_CONTACT_RESPONSE);
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
    
    if (spinApp::Instance().getContext()->isServer())
    {
        if (isDynamic_ && (mass_!=0))
        {
            btScalar m[16];

            btDefaultMotionState* myMotionState = (btDefaultMotionState*) body_->getMotionState();
            myMotionState->m_graphicsWorldTrans.getOpenGLMatrix(m);

            osg::Matrixf mat(m);
            
            this->setTranslation(mat.getTrans().x(),mat.getTrans().y(),mat.getTrans().z());
        }
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


    osg::Matrix mat2 = asOsgMatrix(body_->getWorldTransform());
    std::cout << "   wrld trans: " << stringify(mat2.getTrans()) << std::endl;
    std::cout << "   wrld orien: " << stringify(mat2.getRotate()) << std::endl;
    std::cout << "   wrld scale: " << stringify(mat2.getScale()) << std::endl;

    
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
    // wouldn't it be better to apply translation to the physics here, then
    // check for collisions immediately and only proceed with setTranslation
    // if there wasn't a collision?
    // Would we use btCollisionWorld::ContactResultCallback for this?
    // http://www.bulletphysics.org/mediawiki-1.5.8/index.php/Collision_Callbacks_and_Triggers


    lastBtHitDepth = 0;
    
    ShapeNode::setTranslation(x,y,z);
    
    // TODO: use global matrix
    
    btVector3 pos = asBtVector3(this->getTranslation());
        
    currentTransform_.setOrigin(pos);
    
    //body_->setWorldTransform(currentTransform_);
    body_->getMotionState()->setWorldTransform(currentTransform_);
}

void CollisionShape::setOrientation(float pitch, float roll, float yaw)
{
    ShapeNode::setOrientation(pitch, roll, yaw);
    
    // TODO: use global matrix
    
    btQuaternion quat = asBtQuaternion(this->getOrientationQuat());
    currentTransform_.setRotation(quat);
    
    //body_->setWorldTransform(currentTransform_);
    body_->getMotionState()->setWorldTransform(currentTransform_);
}

void CollisionShape::setScale(float x, float y, float z)
{
    ShapeNode::setScale(x,y,z);
    
    // TODO: use global matrix
    
    //body_->setWorldTransform(currentTransform_);
    collisionObj_->setLocalScaling(asBtVector3(this->getScale()));
}

void CollisionShape::setManipulatorMatrix
    (float a00, float a01, float a02, float a03,
     float a10, float a11, float a12, float a13,
     float a20, float a21, float a22, float a23,
     float a30, float a31, float a32, float a33)
{
    ShapeNode::setManipulatorMatrix(a00,a01,a02,a03,
                                    a10,a11,a12,a13,
                                    a20,a21,a22,a23,
                                    a30,a31,a32,a33);
                                
    currentTransform_ = asBtTransform(mainTransform->getMatrix());
    body_->getMotionState()->setWorldTransform(currentTransform_);
    collisionObj_->setLocalScaling(asBtVector3(this->getScale()));
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

