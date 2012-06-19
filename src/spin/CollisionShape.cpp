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

//extern float lastBtHitDepth;
static float lastBtHitDepth = 0.0;

namespace spin
{


struct btContactCallback : public btCollisionWorld::ContactResultCallback
{
	
	//! Constructor, pass whatever context you want to have available when processing contacts
	/*! You may also want to set m_collisionFilterGroup and m_collisionFilterMask
	 *  (supplied by the superclass) for needsCollision() */
	btContactCallback(btRigidBody& tgtBody , CollisionShape& node /*, ... */)
		: btCollisionWorld::ContactResultCallback(), body(tgtBody), node_(node) { }
	
	btRigidBody& body; //!< The body the sensor is monitoring
	CollisionShape& node_; //!< External information for contact processing
	
	//! If you don't want to consider collisions where the bodies are joined by a constraint, override needsCollision:
	/*! However, if you use a btCollisionObject for #body instead of a btRigidBody,
	 *  then this is unnecessary—checkCollideWithOverride isn't available */
	virtual bool needsCollision(btBroadphaseProxy* proxy) const {
		// superclass will check m_collisionFilterGroup and m_collisionFilterMask
		if(!btCollisionWorld::ContactResultCallback::needsCollision(proxy))
			return false;
		// if passed filters, may also want to avoid contacts between constraints
		return body.checkCollideWithOverride(static_cast<btCollisionObject*>(proxy->m_clientObject));
	}
	
	//! Called with each contact for your own processing (e.g. test if contacts fall in within sensor parameters)
	virtual btScalar addSingleResult(btManifoldPoint& cp,
		const btCollisionObject* colObj0,int partId0,int index0,
		const btCollisionObject* colObj1,int partId1,int index1)
	{
		osg::Vec3 hitPoint;
        osg::Vec3 hitPoint2;
        
        osg::Vec3 normal;
        
		if (colObj0==&body)
        {
			hitPoint = asOsgVec3( cp.m_localPointA );
			hitPoint2 = asOsgVec3( cp.m_localPointB );
            normal = -asOsgVec3( cp.m_normalWorldOnB );
		} else {
			hitPoint = asOsgVec3( cp.m_localPointB );
			hitPoint2 = asOsgVec3( cp.m_localPointA );
            normal = asOsgVec3( cp.m_normalWorldOnB );
		}
        
        CollisionShape *n0 = (CollisionShape*)(colObj0->getUserPointer());
        CollisionShape *n1 = (CollisionShape*)(colObj1->getUserPointer());
        
        // penetration depth
        float depth = cp.getDistance(); 
      
        if (depth==-1) return 0; // ????
        
        //std::cout << "NEW: Hit between " << n0->getID() << " and " << n1->getID() << " at: " << stringify(hitPoint) << ", otherPt: " << stringify(hitPoint2) << ", normal: " << stringify(normal) << ", depth="<< depth << std::endl;
        

        // This collisionOffset is not good because it assumes that the node
        // has actually moved to that collision depth and must be moved back.
        node_.collisionOffset_ = (normal * depth);// + (normal * 0.00001);

        // We need to compute the amount of remaining translation that can be
        // performed before the collision occurs.
        

        /*
        btScalar m[16];
        btDefaultMotionState* myMotionState = (btDefaultMotionState*) body.getMotionState();
        myMotionState->m_graphicsWorldTrans.getOpenGLMatrix(m);
        osg::Matrixf mat(m);

        // vector from current node pos to the position the node would be if we
        // moved it to the collision pos
        osg::Vec3 diff = mat.getTrans() - node_.getTransform()->getMatrix().getTrans();
        
        std::cout << "diff = " << stringify(diff) << std::endl;
        
        // this one worked well:
        node_.collisionOffset_ = -diff + (normal * depth) + (normal * -0.000001);
        */
        
        
        node_.collisionOffset_ = (normal * depth) + (normal * -0.000001);
     


        osg::Vec3 newT = node_.getTranslation() + (normal * depth);// + (normal * 0.00001);
        
        if (depth != lastBtHitDepth)
        {
            BROADCAST(n0, "ssfff", "collide", n1->id->s_name, normal.x(), normal.y(), normal.z());
        }

        lastBtHitDepth = depth;
        
        // Returns false, telling Bullet that we did not modify the contact point
        // properties at all. We would return true if we changed friction or
        // something
		return 0;
    }
};

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
    body_->setCcdMotionThreshold(0.0);
    body_->setContactProcessingThreshold(0.0);
    //body_->setCcdSweptSphereRadius(0.2 * 0.5);
   
    //Collision objects with a callback still have collision response with dynamic rigid bodies. In order to use collision objects as trigger, you have to disable the collision response.
    //mBody->setCollisionFlags(mBody->getCollisionFlags() |btCollisionObject::CF_NO_CONTACT_RESPONSE));
    
    //body_->setCollisionFlags( body_->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK | btCollisionObject::CF_NO_CONTACT_RESPONSE);
    body_->setCollisionFlags( body_->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT | btCollisionObject::CF_NO_CONTACT_RESPONSE);
    body_->setActivationState( DISABLE_DEACTIVATION );

  
  
    sceneManager->dynamicsWorld_->addCollisionObject(body_);

    this->setInteractionMode(GroupNode::DRAG);
}

// destructor
CollisionShape::~CollisionShape()
{
}

// -----------------------------------------------------------------------------
void CollisionShape::callbackUpdate(osg::NodeVisitor* nv)
{
    ShapeNode::callbackUpdate(nv);
    
    
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
    
    lastTick_ = osg::Timer::instance()->tick();
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
    
    osg::Vec3 t;
    osg::Quat q;
    osg::Vec3 s;
    osg::Quat so;
    mat.decompose(t, q, s, so);
    std::cout << "   physics trans: " << stringify(t) << std::endl;
    std::cout << "   physics orien: " << stringify(q) << std::endl;
    std::cout << "   physics scale: " << stringify(s) << std::endl;
    
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

bool CollisionShape::checkCollisions(btTransform tranform)
{
    collisionOffset_ = osg::Vec3(0.0,0.0,0.0);

    if (spinApp::Instance().getContext()->isServer())
    {
        body_->getMotionState()->setWorldTransform(tranform);
        body_->setWorldTransform(tranform);

        sceneManager->dynamicsWorld_->updateSingleAabb(body_);

        btContactCallback contactCallback(*body_, *this);
        sceneManager->dynamicsWorld_->contactTest(body_,contactCallback);
        
        if (collisionOffset_.length()) return true;
        
    }
    
    return false;
}

void CollisionShape::setTranslation (float x, float y, float z)
{
    // We start by applying the translation to the physics only and checking for
    // collisions using Bullet's contactTest() method. The callback will set the
    // appropriate contactOffset if there is a collision. Thus, we avoid sending
    // an update to the viewer if there is about to be a collision.
    // See the discussion about btCollisionWorld::ContactResultCallback here:
    // http://www.bulletphysics.org/mediawiki-1.5.8/index.php/Collision_Callbacks_and_Triggers


/*
    collisionOffset_ = osg::Vec3(0.0,0.0,0.0);

    std::cout << getID() << "setTranslation from " << stringify(getTranslation()) << " to " << stringify(osg::Vec3(x,y,z)) << std::endl;

    if (spinApp::Instance().getContext()->isServer())
    {
        btTransform tmpTransform = currentTransform_;
        tmpTransform.setOrigin(btVector3(x,y,z));
        body_->getMotionState()->setWorldTransform(tmpTransform);
        body_->setWorldTransform(tmpTransform);

        osg::Timer_t tick = osg::Timer::instance()->tick();
        double dt =osg::Timer::instance()->delta_s(lastTick_,tick);
        // we don't need to run collision detection on the whole world, right?:
        //sceneManager->dynamicsWorld_->performDiscreteCollisionDetection();
        //sceneManager->dynamicsWorld_->stepSimulation(dt);
        //sceneManager->dynamicsWorld_->updateAabbs();
        
        sceneManager->dynamicsWorld_->updateSingleAabb(body_);

        if (1)
        {
            btScalar m[16];
            btDefaultMotionState* myMotionState = (btDefaultMotionState*) body_->getMotionState();
            myMotionState->m_graphicsWorldTrans.getOpenGLMatrix(m);
            osg::Matrixf mat(m);

            std::cout << "about to callback. phys pos: " << stringify(mat.getTrans()) << std::endl;
            std::cout << "          physics world pos: " << stringify(asOsgMatrix(body_->getWorldTransform()).getTrans()) << std::endl;
        }
        
        //if (body_->checkCollideWithOverride(<#btCollisionObject *co#>)

        btContactCallback contactCallback(*body_, *this);
        sceneManager->dynamicsWorld_->contactTest(body_,contactCallback);
        
        lastTick_ = tick;
    }
    
    std::cout << "done callback for " << stringify(osg::Vec3(x,y,z)) << " collisionOffset: " << stringify(collisionOffset_) << std::endl;
    */
    
    btTransform tmpTransform = currentTransform_;
    tmpTransform.setOrigin(btVector3(x,y,z));
    if (checkCollisions(tmpTransform))
        ShapeNode::setTranslation(x+collisionOffset_.x(), y+collisionOffset_.y(), z+collisionOffset_.z());
        //ShapeNode::setTranslation(collisionOffset_.x(), collisionOffset_.y(), collisionOffset_.z());
    else
        ShapeNode::setTranslation(x, y, z);
    
    currentTransform_.setOrigin(asBtVector3(this->getTranslation()));
    body_->getMotionState()->setWorldTransform(currentTransform_);
    body_->setWorldTransform(currentTransform_);
    

    /* old
    
    
    lastBtHitDepth = 0;
    
    ShapeNode::setTranslation(x,y,z);
    
    // TODO: use global matrix
    
    btVector3 pos = asBtVector3(this->getTranslation());
        
    currentTransform_.setOrigin(pos);
    
    //body_->setWorldTransform(currentTransform_);
    body_->getMotionState()->setWorldTransform(currentTransform_);
    
    */
}

void CollisionShape::setOrientationQuat(float x, float y, float z, float w)
{
    btTransform tmpTransform = currentTransform_;
    tmpTransform.setRotation(btQuaternion(x,y,z,w));
    if (checkCollisions(tmpTransform))
    {
        ShapeNode::translate(collisionOffset_.x(), collisionOffset_.y(), collisionOffset_.z());
    }

    ShapeNode::setOrientationQuat(x, y, z, w);
    btQuaternion quat = asBtQuaternion(this->getOrientationQuat());
    currentTransform_.setRotation(quat);
    body_->getMotionState()->setWorldTransform(currentTransform_);
}

void CollisionShape::setOrientation(float pitch, float roll, float yaw)
{
    osg::Quat newQ = osg::Quat( osg::DegreesToRadians(pitch), osg::Vec3d(1,0,0),
                                osg::DegreesToRadians(roll), osg::Vec3d(0,1,0),
                                osg::DegreesToRadians(yaw), osg::Vec3d(0,0,1));
    btTransform tmpTransform = currentTransform_;
    tmpTransform.setRotation(asBtQuaternion(newQ));
    if (checkCollisions(tmpTransform))
    {
        ShapeNode::translate(collisionOffset_.x(), collisionOffset_.y(), collisionOffset_.z());
    }




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
    
    
    //currentTransform_ = asBtTransform(mainTransform->getMatrix());
    //body_->getMotionState()->setWorldTransform(currentTransform_);
    
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
    return;
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

