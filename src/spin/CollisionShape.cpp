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
#include <osgDB/WriteFile>
#include <osgDB/ReadFile>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>

#include <bullet/BulletCollision/Gimpact/btGImpactShape.h>

#include "collisionshape.h"
#include "spinapp.h"
#include "spinbasecontext.h"
#include "scenemanager.h"
#include "osgutil.h"
#include "bulletutil.h"

using namespace std;

//extern float lastBtHitDepth;
static float lastBtHitDepth = 0.0;

extern ContactAddedCallback gContactAddedCallback; 

#define LINEAR_SLEEPING_THRESHOLD 0.05
#define ANGULAR_SLEEPING_THRESHOLD 0.1

namespace spin
{

static bool callbackFunc(btManifoldPoint& cp,const btCollisionObject* obj1,int id1,int index1,const btCollisionObject* obj2,int id2,int index2)
{
    //((bulletObject*)obj1->getUserPointer())->hit=true;
    //   ((bulletObject*)obj2->getUserPointer())->hit=true;
    printf("callbackFunc!\n");
        return false;
}

//struct btContactCallback : public btCollisionWorld::ContactResultCallback
//{

    //! Constructor, pass whatever context you want to have available when processing contacts
    /*! You may also want to set m_collisionFilterGroup and m_collisionFilterMask
     *  (supplied by the superclass) for needsCollision() */

CollisionShape::btContactCallback::btContactCallback(btRigidBody& tgtBody , CollisionShape& node /*, ... */)
        : btCollisionWorld::ContactResultCallback(), body(tgtBody), node_(node), prevHit(false), hit(false) { }

CollisionShape::btContactCallback::~btContactCallback() {}

    /*    btRigidBody& body; //!< The body the sensor is monitoring
    CollisionShape& node_; //!< External information for contact processing

    osg::Vec3 prevHitPoint;
    osg::Vec3 prevHitPoint2;
    const btCollisionObject* prevObj;
    bool hit, prevHit;
    */
    //! If you don't want to consider collisions where the bodies are joined by a constraint, override needsCollision:
    /*! However, if you use a btCollisionObject for #body instead of a btRigidBody,
     *  then this is unnecessary—checkCollideWithOverride isn't available */
bool CollisionShape::btContactCallback::needsCollision(btBroadphaseProxy* proxy) const
{
    // superclass will check m_collisionFilterGroup and m_collisionFilterMask
    if(!btCollisionWorld::ContactResultCallback::needsCollision(proxy))
        return false;
    // if passed filters, may also want to avoid contacts between constraints
    return body.checkCollideWithOverride(static_cast<btCollisionObject*>(proxy->m_clientObject));
}

//! Called with each contact for your own processing (e.g. test if contacts fall in within sensor parameters)
btScalar CollisionShape::btContactCallback::addSingleResult(btManifoldPoint& cp,
                                                            const btCollisionObject* colObj0,int partId0,int index0,
                                                            const btCollisionObject* colObj1,int partId1,int index1)
                                                            // const btCollisionObjectWrapper* colObj0Wrap,int partId0,int index0,
                                                            // const btCollisionObjectWrapper* colObj1Wrap,int partId1,int index1)
{
    osg::Vec3 hitPoint;
    osg::Vec3 hitPoint2;
    osg::Vec3 normal;

    // const btCollisionObject* colObj0 = colObj0Wrap->getCollisionObject();
    // const btCollisionObject* colObj1 = colObj1Wrap->getCollisionObject();
    const btCollisionObject* otherObj;
    const btCollisionObject* thisObj;
    if (colObj0==&body) {
        hitPoint = asOsgVec3( cp.m_localPointA );
        hitPoint2 = asOsgVec3( cp.m_localPointB );
        normal = -asOsgVec3( cp.m_normalWorldOnB );// there is no m_normalWorldOnA
        thisObj = colObj0;
        otherObj = colObj1;
    } else {
        hitPoint = asOsgVec3( cp.m_localPointB );
        hitPoint2 = asOsgVec3( cp.m_localPointA );
        normal = asOsgVec3( cp.m_normalWorldOnB );
        thisObj = colObj1;
        otherObj = colObj0;
    }

    hit = true;
    if ( prevHit && otherObj == prevObj ) {
        //printf("prevHit\n");
        return 0;
    } 

    /*if ( (hitPoint == prevHitPoint || hitPoint2 == prevHitPoint2) && colObj1 == prevObj ) {
        //printf("nope\n");
        return 0;
        }*/
    prevHitPoint = hitPoint;
    prevHitPoint2 = hitPoint2;
    prevObj = otherObj;
    
    CollisionShape *n0 = (CollisionShape*)(thisObj->getUserPointer());
    CollisionShape *n1 = (CollisionShape*)(otherObj->getUserPointer());
    //CollisionShape *n0 = (CollisionShape*)(colObj0->getCollisionObject()->getUserPointer());
    //CollisionShape *n1 = (CollisionShape*)(colObj1->getCollisionObject()->getUserPointer());


    if ( !n0 || !n1 ) return 0;

    if ( !n0->getNodeSymbol() ) { /*printf( "contactcallback: n0 id_ null\n" );*/ return 0;}
    if ( !n1->getNodeSymbol() ) { /*printf( "contactcallback: n1 id_ null\n" );*/ return 0;}
    //     if ( n0->getNodeSymbol() && !n0->getID() ) {printf( "contactcallback: n0 s_name null\n" ); return 0;}
    //if ( n1->getNodeSymbol() && !n0->getID() ) {printf( "contactcallback: n1 s_name null\n" ); return 0;}


    // penetration depth
    float depth = cp.getDistance();

    if (depth==-1) return 0; // ????

    std::cout << "NEW: Hit between " << n0->getID() << " and " << n1->getID() << " at: " << stringify(hitPoint) << ", otherPt: " << stringify(hitPoint2) << ", normal: " << stringify(normal) << ", depth="<< depth << std::endl;


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


    btScalar impulse = cp.getAppliedImpulse();

    printf("collide 0x%p %s, %f %f %f, %f %f %f, %f\n", &cp, n1->getID().c_str(),
           hitPoint.x(), hitPoint.y(), hitPoint.z(),
           normal.x(), normal.y(), normal.z(), cp.m_appliedImpulse);

    if (depth != lastBtHitDepth)
    {
        BROADCAST( n0, "ssfffffff", "collide", n1->getID().c_str(),
                   hitPoint.x(), hitPoint.y(), hitPoint.z(),
                   normal.x(), normal.y(), normal.z(), impulse );
    }

    lastBtHitDepth = depth;

    // Returns false, telling Bullet that we did not modify the contact point
    // properties at all. We would return true if we changed friction or
    // something
    return 0;
}
////};

// -----------------------------------------------------------------------------
// constructor:
CollisionShape::CollisionShape (SceneManager *sceneManager, const char* initID) : ShapeNode(sceneManager, initID)
{
    this->setName(this->getID() + ".CollisionShape");
    this->setNodeType("CollisionShape");

    mass_ = 1.0;
    isDynamic_ = false;
    bounciness_ = 0.0; // bt default
    friction_ = 0.5; // bt default
    rollingFriction_ = 0.5;// not bt default, but more practical, so that round things don't roll forever.

    //gContactAddedCallback = btCollisionCallback2;
        
    collisionShp_ = new btBoxShape( btVector3(0.5,0.5,0.5) );

    currentTransform_.setIdentity();

    btVector3 localInertia(0,0,0);
    if (mass_!=0.f) collisionShp_->calculateLocalInertia(mass_, localInertia);


    // using motionstate is recommended, it provides interpolation capabilities,
    // and only synchronizes 'active' objects
    motionState_ = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0)));//currentTransform_);
    btRigidBody::btRigidBodyConstructionInfo rbci(mass_, motionState_, collisionShp_, localInertia);
    rbci.m_restitution = bounciness_;
    rbci.m_friction = friction_;
    //BULLET VERSION >= 2.81 rbci.m_rollingFriction = rollingFriction_;
    rbci.m_angularDamping = rollingFriction_; // WILL DO FOR NOW
    body_ = new btRigidBody(rbci);
    body_->setUserPointer(this);

    // body_->setCcdMotionThreshold(0.1);
    // body_->setCcdSweptSphereRadius(0.05);


    //Collision objects with a callback still have collision response with dynamic rigid bodies. In order to use collision objects as trigger, you have to disable the collision response.
    //mBody->setCollisionFlags(mBody->getCollisionFlags() |btCollisionObject::CF_NO_CONTACT_RESPONSE));

    //body_->setCollisionFlags( body_->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK | btCollisionObject::CF_NO_CONTACT_RESPONSE);
    //body_->setCollisionFlags( btCollisionObject::CF_KINEMATIC_OBJECT );
    //body_->setCollisionFlags( btCollisionObject::CF_NO_CONTACT_RESPONSE );
    body_->setCollisionFlags( 0 );
    //printf("getCollisionFlags = 0x%x\n", body_->getCollisionFlags()  );
    //body_->setActivationState( DISABLE_DEACTIVATION );

    //    sceneManager_->dynamicsWorld_->addCollisionObject(body_);
    body_->setSleepingThresholds( LINEAR_SLEEPING_THRESHOLD, ANGULAR_SLEEPING_THRESHOLD );
    //    body_->setSleepingThresholds( 1.6, 2.5 ); //?
    // body_->setDeactivationTime(0.8);

    /////////////////////
    //// NNOT IF IT STARTS AS NOT DYNAMIC
    /////////////////////////////sceneManager_->dynamicsWorld_->addRigidBody(body_);
    ////////////////////

    // hit_ = false;
    // prevHit_ = false;
    // prevHitDepth_ = 1000000;
    windFactor_ = 1.0f;
    reportContacts_ = false;
    filterContacts_ = true;
    contactCallback_ = 0; //new btContactCallback( *body_, *this );
    modelGeometry_ = 0;
    this->setInteractionMode(GroupNode::DRAG);
}

// destructor
CollisionShape::~CollisionShape()
{

    btTypedConstraint* c;
    btConstraints::iterator it;
    for ( it = constraints_.begin(); it != constraints_.end(); it++ ) {
        c = it->second;
        sceneManager_->dynamicsWorld_->removeConstraint( c );
        if ( c ) delete( c );
    }

    sceneManager_->dynamicsWorld_->removeRigidBody(body_);
    delete body_->getMotionState();
    delete body_;

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
        //printf("callbackUpdate %s ---\n", getID().c_str());

        // prevHit_ = hit_;
        // hit_ = false;
        
        Colliders::iterator it;
        for ( it = colliders_.begin(); it != colliders_.end(); it++ ) {
            it->second++;
            //printf(" -- %s: %i\n", it->first.c_str(), it->second );
            if ( it->second > MAX_COLLISION_LIFETIME ) colliders_.erase( it->first );
        }




        /* {
                btDefaultMotionState* myMotionState = (btDefaultMotionState*) body_->getMotionState();
                myMotionState->getWorldTransform( currentTransform_ );
                std::cout << getID() << " transform C = " << stringify(asOsgVec3( currentTransform_.getOrigin()))<< std::endl;
                }*/
        /*std::cout << getID() ;
          printf(" state = %i; time = %f, s %i; k %i; ", body_->getActivationState(), body_->getDeactivationTime(),
               body_->isStaticObject(), body_->isKinematicObject() );
        printf( "velocities: %f, %f\n", body_->getLinearVelocity().length2(), body_->getAngularVelocity().length2() );

        if ((body_->getLinearVelocity().length2() < body_->getLinearSleepingThreshold()*body_->getLinearSleepingThreshold()) &&
            (body_->getAngularVelocity().length2() < body_->getAngularSleepingThreshold()*body_->getAngularSleepingThreshold() )) {
            std::cout << getID(); printf(" sleeping threshold!!\n");
            }*/

        if (isDynamic_ && (mass_!=0) && body_->isInWorld() && body_->isActive() )
        {
            // if (contactCallback_) {
            //     //printf("\n%s **************\n", this->getID().c_str());
            //     if ( filterContacts_ ) contactCallback_->hit = false;
            //     else contactCallback_->prevHit = false;
            //     sceneManager_->dynamicsWorld_->contactTest( body_, *contactCallback_ );
            //     contactCallback_->prevHit = contactCallback_->hit;
            // }


            /*{
                btDefaultMotionState* myMotionState = (btDefaultMotionState*) body_->getMotionState();
                myMotionState->getWorldTransform( currentTransform_ );
                std::cout << getID() << " transform D = " << stringify(asOsgVec3( currentTransform_.getOrigin()))<< std::endl;
                }*/

            btScalar m[16];
            btDefaultMotionState* myMotionState = (btDefaultMotionState*) body_->getMotionState();
            myMotionState->m_graphicsWorldTrans.getOpenGLMatrix(m);
            osg::Matrixf mat(m);

            /* printf("\n%s\n", getID().c_str());
            printf("CollisionShape::callbackUpdate setTranslation(%f, %f, %f)\n", mat.getTrans().x(),mat.getTrans().y(),mat.getTrans().z());
            printf("                               setOrientationQuat(%f, %f, %f, %f)\n", mat.getRotate().x(), mat.getRotate().y(), mat.getRotate().z(), mat.getRotate().w() );*/
            //   std::cout << getID();
            //   printf("ShapeNode::setTranslation %f %f %f\n", mat.getTrans().x(),mat.getTrans().y(),mat.getTrans().z() );
            ShapeNode::setTranslation(mat.getTrans().x(),mat.getTrans().y(),mat.getTrans().z());
            ShapeNode::setOrientationQuat(  mat.getRotate().x(), mat.getRotate().y(), mat.getRotate().z(), mat.getRotate().w() );
            myMotionState->getWorldTransform( currentTransform_ );
            //  std::cout << getID() << " transform E = " << stringify( asOsgVec3(currentTransform_.getOrigin()))<< std::endl;
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

void CollisionShape::resetCollisionObj()
{
    bool isInWorld = body_->isInWorld();
    if ( isInWorld ) sceneManager_->dynamicsWorld_->removeRigidBody(body_);

    delete( body_ );
    if ( !collisionShp_ ) return;

    btVector3 localInertia(0,0,0);
    if (mass_!=0.f) collisionShp_->calculateLocalInertia(mass_, localInertia);

    btRigidBody::btRigidBodyConstructionInfo rbci(mass_, motionState_, collisionShp_, localInertia);
    rbci.m_restitution = bounciness_;
    rbci.m_friction = friction_;
    //BULLET VERSION >= 2.81 rbci.m_rollingFriction = rollingFriction_;
    rbci.m_angularDamping = rollingFriction_; // WILL DO FOR NOW

    body_ = new btRigidBody(rbci);
    body_->setUserPointer(this);
    body_->setSleepingThresholds( LINEAR_SLEEPING_THRESHOLD, ANGULAR_SLEEPING_THRESHOLD );
    
    if ( isDynamic_ ) {
        if ( mass_ == 0.f )  body_->setCollisionFlags( btCollisionObject::CF_KINEMATIC_OBJECT ); 
        else body_->setCollisionFlags( 0 );
    } else {
        //if ( mass_ == 0.f ) body_->setCollisionFlags( btCollisionObject::CF_NO_CONTACT_RESPONSE );
        //else body_->setCollisionFlags( btCollisionObject::CF_KINEMATIC_OBJECT );
        //body_->setCollisionFlags( btCollisionObject::CF_NO_CONTACT_RESPONSE );
    }

    if ( isInWorld )  sceneManager_->dynamicsWorld_->addRigidBody(body_);

}


// -----------------------------------------------------------------------------


void CollisionShape::setShape( shapeType t )
{

    if (t == shape) return;
	else shape = t;

    bool isInWorld = body_->isInWorld();
    if ( isInWorld ) sceneManager_->dynamicsWorld_->removeRigidBody(body_);

    switch (t) {
    case NONE:
        isInWorld = false;
        if (collisionShp_) delete collisionShp_;
        break;
    case MODEL:
        //printf("model!\n");
        break;
    case SPHERE:
        collisionShp_ = new btSphereShape( AS_UNIT_SCALE * 0.5 );
        break;
    case CYLINDER:
        collisionShp_ = new btCylinderShapeZ( AS_UNIT_SCALE * btVector3(0.25,0.25,0.5) );
        break;
    case CAPSULE:
        collisionShp_ = new btCapsuleShape( AS_UNIT_SCALE * 0.25, AS_UNIT_SCALE );
        break;
    case CONE:
        collisionShp_ = new btConeShape( AS_UNIT_SCALE * 0.25, AS_UNIT_SCALE );
        break;
    case PLANE:
        collisionShp_ = new btStaticPlaneShape( btVector3(0, 1, 0), 0 ); // 'createPlane' makes a y=0 plane.
        break;
    case BOX:
    default:
        collisionShp_ = new btBoxShape( AS_UNIT_SCALE * btVector3(0.5,0.5,0.5) );
        break;
    }

    /* btVector3 localInertia(0,0,0);
    if (mass_!=0.f) collisionShp_->calculateLocalInertia(mass_, localInertia);

    ///////recycle old.motionState_ = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0)));//currentTransform_);
    btRigidBody::btRigidBodyConstructionInfo rbci(mass_, motionState_, collisionShp_, localInertia);
    rbci.m_restitution = bounciness_;
    rbci.m_friction = friction_;
    body_ = new btRigidBody(rbci);



    body_->setUserPointer(this);
    //printf("getCollisionFlags = 0x%x\n", body_->getCollisionFlags()  );  
    body_->setCollisionFlags( btCollisionObject::CF_KINEMATIC_OBJECT );
    //printf("getCollisionFlags = 0x%x\n", body_->getCollisionFlags()  );
    */
    if ( shape != MODEL && modelGeometry_.get() ) modelGeometry_.release();
   
	drawShape();

    if ( modelGeometry_.get() && shape == MODEL && shapeGeode.get() ) {

        //if (!shapeGeode) shapeGeode = new osg::Geode();
        shapeGeode->addDrawable( modelGeometry_ );
        //if ( !this->getAttachmentNode()->containsNode(shapeGeode.get()) )
        //    this->getAttachmentNode()->addChild(shapeGeode.get());
		//shapeGeode->setName(this->getID() + ".shapeGeode");
		//optimizer.optimize(shapeGeode.get());
    }

    resetCollisionObj();


    if ( isInWorld )  sceneManager_->dynamicsWorld_->addRigidBody(body_);

    if ( !broadcastLock_ ) BROADCAST(this, "si", "setShape", (int) shape);
}

// -----------------------------------------------------------------------------

static btCollisionShape* osgGeom2Bullet( osg::Geometry* geom )
{
    osg::Vec3Array* vert = (osg::Vec3Array*) geom->getVertexArray();
    osg::PrimitiveSet* ps =  geom->getPrimitiveSet(0);
    if ( !ps ) {
        printf( "no primitiveset\n" );
        return 0;
    }

    osg::DrawElementsUInt* de = (osg::DrawElementsUInt*) ps->getDrawElements();

    //printf("osgGeom2Bullet... %i %i\n",  de->getNumIndices(), vert->getNumElements());
    
    int* index = (int*)malloc( de->getNumIndices()*sizeof(int));
    for ( size_t i = 0; i < de->getNumIndices(); i++ )
        index[i] = (*de)[i];
    
    btScalar* vertss = (btScalar*)malloc( vert->getNumElements()*sizeof(btScalar)*3 );
    for ( size_t i = 0; i < vert->getNumElements(); i++ ) {
        vertss[i*3+0] = (*vert)[i].x();
        vertss[i*3+1] = (*vert)[i].y();
        vertss[i*3+2] = (*vert)[i].z();
    }
    /*
	btTriangleIndexVertexArray* btmesh =
        new btTriangleIndexVertexArray( de->getNumIndices()/3,
                                        //(int*)de->getDataPointer(),
                                        &index[0],
                                        //3*sizeof(unsigned int),//!!!!!!!!!!!! bad int types
                                        3*sizeof(int),
                                        vert->getNumElements(),
                                        &vertss[0],//(btScalar*)vert->getDataPointer(),
                                        sizeof(btScalar)*3 );
    printf("heyyy\n");
    btVector3 aabbMin(-1000,-1000,-1000),aabbMax(1000,1000,1000);
    bool useQuantizedAabbCompression = true;
    printf("heyyyy\n");*/
    btCollisionShape* trimeshShape;
    trimeshShape = new btConvexHullShape( &vertss[0], vert->getNumElements(), sizeof(btScalar)*3 );
        //new btBvhTriangleMeshShape( btmesh, useQuantizedAabbCompression, aabbMin, aabbMax ); 

    // new btGIMPACTMeshData(m_indexVertexArrays);  
    //trimeshShape = new btGImpactMeshShape( btmesh );

    return trimeshShape;
}

void CollisionShape::setModelFromFile( const char* file )
{
        
    osgDB::Registry::instance()->getDataFilePathList().push_back( osgDB::getFilePath( getAbsolutePath(std::string(file)) ) );
    std::string modelPath = getRelativePath(std::string(file));
    osg::Group* group = (osg::Group*)(osgDB::readNodeFile( getAbsolutePath(modelPath).c_str() ));
    
    if (!group) {
        printf("loading of file [%s] failed!\n", modelPath.c_str());
        return;
    }

    osg::Geode* geode;
    osg::Geometry* geom;
    for ( size_t i = 0; i < group->getNumChildren(); i++ ) {
        geode = dynamic_cast<osg::Geode*>( group->getChild(i) );
        if ( !geode ) continue;
        geom = geode->getDrawable(0)->asGeometry();
        if ( geom ) break;
    }

    if ( !geom ) return;
    
    osg::Vec4Array* c = dynamic_cast<osg::Vec4Array*>( geom->getColorArray() );
    if ( c ) {
        if ( c->getNumElements() == 1 ) (*c)[0].set( _color.r(), _color.g(), _color.b(), _color.a() );
    } else {
        c = new osg::Vec4Array();
        (*c).push_back( _color );
        geom->setColorArray(c);
        geom->setColorBinding(osg::Geometry::BIND_OVERALL);
    }

    collisionShp_ = osgGeom2Bullet( geom );
    
	osg::TessellationHints* hints = new osg::TessellationHints;
    hints->setDetailRatio(detailRatio_);
    modelGeometry_ =// new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.0f,0.0f,0.0f),AS_UNIT_SCALE*.5), hints);
        geom;
    setBroadcastLock( true );
    setShape( MODEL );
    setBroadcastLock( false );
    //osgDB::writeNodeFile( *this->getAttachmentNode(), "bouse.osg" );

    BROADCAST(this, "ss", "setModelFromFile", file);
}

// -----------------------------------------------------------------------------

void CollisionShape::addConstraint( const char* lbl, float x, float y, float z )
{
    removeConstraint( lbl );
    btTypedConstraint* c = new btPoint2PointConstraint( *body_, btVector3(x,y,z) );

    constraints_[lbl] = c;
    if ( isDynamic_ ) sceneManager_->dynamicsWorld_->addConstraint( c );
    //printf("yea %f %f %f\n",x,y,z);
}

void CollisionShape::addConstraint2(  const char* lbl, float x, float y, float z,
                                      const char* otherObj, float ox, float oy, float oz )
{
    ReferencedNode* otherNode = sceneManager_->getNode( otherObj );
    if ( !otherNode ) return;

    CollisionShape* otherShape = dynamic_cast<CollisionShape*>(otherNode);
    if ( !otherShape ) return;
    
    removeConstraint( lbl );
    btTypedConstraint* c = new btPoint2PointConstraint( *body_, *otherShape->getBody(),
                                                        btVector3(x,y,z), btVector3(ox,oy,oz) );
    constraints_[lbl] = c;
    if ( isDynamic_ ) sceneManager_->dynamicsWorld_->addConstraint( c, true );// true:joined object do not collide with each other

}

void CollisionShape::removeConstraint( const char* lbl )
{

    btTypedConstraint* c;
    btConstraints::iterator it = constraints_.find( lbl );
    if ( it != constraints_.end() ) {
        c = it->second;
        sceneManager_->dynamicsWorld_->removeConstraint( c );
        if ( c ) delete( c );
    }
    constraints_.erase( lbl );
}

// -----------------------------------------------------------------------------
void CollisionShape::wakeup()
{
    if ( !isDynamic_ || !body_->isInWorld() ||
         !spinApp::Instance().getContext()->isServer() ) return;

    std::cout << getID(); printf("wakeup!\n");
    body_->activate();

    /*    btTypedConstraint* c;
    btConstraints::iterator it;
    for ( it = constraints_.begin(); it != constraints_.end(); it++ ) {
        printf("constraint [%s]\n", it->first.c_str());
        c = it->second;
        btRigidBody& r = c->getRigidBodyB();        
        if ( !r.isInWorld() ) continue;
        CollisionShape* cs = (CollisionShape*) r.getUserPointer();
        if ( !cs ) continue;
        std::cout << " ... " <<  cs->getID() << std::endl;;
        if ( cs->isDynamic_ ) r.activate();
        }*/
}

// -----------------------------------------------------------------------------

void CollisionShape::setBounciness( float f )
{    
    if ( bounciness_ == (btScalar)f ) return;
    //printf("setBounciness %f\n", f);
    bounciness_ = f;
    resetCollisionObj();
    BROADCAST(this, "sf", "setBounciness", (float) bounciness_);
}

// -----------------------------------------------------------------------------

void CollisionShape::setFriction( float f )
{
    if ( friction_ == (btScalar)f ) return;
    //printf("setFriction %f\n", f);
    friction_ = f;
    resetCollisionObj();
    BROADCAST(this, "sf", "setFriction", (float) friction_);
}
// -----------------------------------------------------------------------------

void CollisionShape::setRollingFriction( float f )
{
    if ( friction_ == (btScalar)f ) return;
    //printf("setRollingFriction %f\n", f);
    rollingFriction_ = f;
    resetCollisionObj();
    BROADCAST(this, "sf", "setFriction", (float) friction_);
}// -----------------------------------------------------------------------------

void CollisionShape::setWindFactor( float f )
{
    if ( windFactor_ == (btScalar)f ) return;
    //printf("setRollingFriction %f\n", f);
    windFactor_ = f;
    
    BROADCAST(this, "sf", "setWindFactor", (float) windFactor_);
}
// -----------------------------------------------------------------------------

void CollisionShape::reportContact( btManifoldPoint& cp, const btCollisionObject* otherObj, bool swap )
{
    if ( !reportContacts_ ) return;

    CollisionShape *cs = (CollisionShape*)(otherObj->getUserPointer());
    if ( !cs ) return;

    if ( filterContacts_ ) {
        Colliders::iterator it = colliders_.find(cs->getID());
        if ( it != colliders_.end() && it->second < MIN_COLLISION_LIFETIME ) {
            it->second = 0;
            //printf("prevented hit on %s", it->first.c_str());
            return;
        }
    } 

    // else {
    //         prevHit_ = false;
    //     } 

    //     hit_ = true;

    // if ( prevHit_ && otherObj == prevHitObj_ ) {
    //     //printf("prevHit\n");
    //     return;
    // } 

    osg::Vec3 hitPoint;
    osg::Vec3 normal;

    if ( swap ) {
        hitPoint = asOsgVec3( cp.m_localPointB );
        normal = asOsgVec3( cp.m_normalWorldOnB );
    } else {
        hitPoint = asOsgVec3( cp.m_localPointA );
        normal = -asOsgVec3( cp.m_normalWorldOnB );// there is no m_normalWorldOnA
    }

    float depth = cp.getDistance();
    //prevHitObj_ = otherObj;   
    
    btScalar impulse = cp.getAppliedImpulse();


    colliders_[cs->getID()] = 0;

    //if ( depth != prevHitDepth_ ) {
        
    // printf( "%s collide %s, %f %f %f, %f %f %f, %f\n",
    //          getID().c_str(), cs->getID().c_str(),
    //          hitPoint.x(), hitPoint.y(), hitPoint.z(),
    //          normal.x(), normal.y(), normal.z(), impulse );


    BROADCAST( this, "ssfffffff", "collide", cs->getID().c_str(),
               hitPoint.x(), hitPoint.y(), hitPoint.z(),
               normal.x(), normal.y(), normal.z(), impulse );

    //prevHitDepth_ = depth;

    //setColor( 1.0f-_color.r(), 1.0f-_color.g(), 1.0f-_color.b(), _color.a() );


    

}


// -----------------------------------------------------------------------------

void CollisionShape::setReportContacts( int b )
{

    // if ( (!b) == (!contactCallback_) ) return;

    // if ( b ) contactCallback_ = new btContactCallback( *body_, *this );
    // else contactCallback_ = 0;

    if ( b == reportContacts_ ) return;
    reportContacts_ = b;    

    BROADCAST(this, "si", "setReportContacts", b);
}

// -----------------------------------------------------------------------------
void CollisionShape::setFilterContacts( int b )
{
    if ( b == filterContacts_ ) return;
    filterContacts_ = b;
    BROADCAST(this, "si", "setFilterContacts", b);
}

// -----------------------------------------------------------------------------

void CollisionShape::setMass (float mass)
{
    mass_ = mass;

    btVector3 localInertia(0, 0, 0);

    // rigidbody is dynamic if and only if mass is non zero, otherwise static
    //bool hasMass = (mass_ != 0.f);
    //if ( hasMass )
    //    collisionShp_->calculateLocalInertia(mass_, localInertia);
    bool isInWorld = body_->isInWorld();
    if ( isInWorld ) sceneManager_->dynamicsWorld_->removeRigidBody(body_);

    collisionShp_->calculateLocalInertia(mass_, localInertia);
    body_->setMassProps(mass_, localInertia);

    ///if ( mass_ == 0.0f && isDynamic_ ) body_->setCollisionFlags( btCollisionObject::CF_KINEMATIC_OBJECT );
    if ( isDynamic_ ) {
        if ( mass_ == 0.f )  body_->setCollisionFlags( btCollisionObject::CF_KINEMATIC_OBJECT ); 
        else body_->setCollisionFlags( 0 );
    }
    // else {
    //     body_->setCollisionFlags( btCollisionObject::CF_NO_CONTACT_RESPONSE );
    // }

    if ( isInWorld )  sceneManager_->dynamicsWorld_->addRigidBody(body_);

    BROADCAST(this, "sf", "setMass", getMass());
}

// -----------------------------------------------------------------------------
void CollisionShape::setDynamic(int isDynamic)
{
    if (isDynamic_ == isDynamic) return;
    isDynamic_ = isDynamic;
    ///if ( body_->isInWorld() ) sceneManager_->dynamicsWorld_->removeRigidBody(body_);
    if (isDynamic_) {
        //if ( checkCollisions(currentTransform_) )
        //    ShapeNode::translate(collisionOffset_.x(), collisionOffset_.y(), collisionOffset_.z());
        //std::cout << getID(); printf(" dynamic!\n");
        if ( mass_ == 0.0f ) body_->setCollisionFlags( btCollisionObject::CF_KINEMATIC_OBJECT ); 
        else body_->setCollisionFlags( 0 );
        
        sceneManager_->dynamicsWorld_->addRigidBody(body_);

        btTypedConstraint* c;
        btConstraints::iterator it;
        for ( it = constraints_.begin(); it != constraints_.end(); it++ ) {
            c = it->second;
            if ( c ) sceneManager_->dynamicsWorld_->addConstraint( c, true );
        }

        body_->activate();

    } else {


        btTypedConstraint* c;
        btConstraints::iterator it;
        for ( it = constraints_.begin(); it != constraints_.end(); it++ ) {
            c = it->second;
            if ( c ) sceneManager_->dynamicsWorld_->removeConstraint( c );
        }

        sceneManager_->dynamicsWorld_->removeRigidBody(body_);



    }



    // else {
    //     body_->setCollisionFlags( btCollisionObject::CF_NO_CONTACT_RESPONSE );
    // }
    //if ( !body_->isInWorld() ) 


    //printf("%s: getCollisionFlags = 0x%x\n", getID().c_str(), body_->getCollisionFlags()  );
    BROADCAST(this, "si", "setDynamic", getDynamic());
}

// -----------------------------------------------------------------------------
bool CollisionShape::checkCollisions(btTransform tranform)
{
    collisionOffset_ = osg::Vec3(0.0,0.0,0.0);

    if (spinApp::Instance().getContext()->isServer())
    {
        if ( !body_->isInWorld() || !body_->getBroadphaseHandle() || !body_->hasContactResponse() ) return false;
        body_->getMotionState()->setWorldTransform(tranform);
        //body_->setWorldTransform(tranform);
        // body_->activate();
        if ( isDynamic_ ) body_->activate();
        sceneManager_->dynamicsWorld_->updateSingleAabb(body_);

  {
        btScalar m[16];
        btDefaultMotionState* myMotionState = (btDefaultMotionState*) body_->getMotionState();
        myMotionState->m_graphicsWorldTrans.getOpenGLMatrix(m);
        osg::Matrixf mat(m);
        std::cout << getID(); printf("checkCollisions %f %f %f\n", mat.getTrans().x(),mat.getTrans().y(),mat.getTrans().z() );
    }

        btContactCallback contactCallback(*body_, *this);
        sceneManager_->dynamicsWorld_->contactTest(body_,contactCallback);

        if (collisionOffset_.length()) return true;

    }

    return false;
}

// -----------------------------------------------------------------------------
void CollisionShape::setTranslation (float x, float y, float z)
{
    //printf("CollisionShape::setTranslation(%f, %f ,%f)\n", x, y, z);
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
        //sceneManager_->dynamicsWorld_->performDiscreteCollisionDetection();
        //sceneManager_->dynamicsWorld_->stepSimulation(dt);
        //sceneManager_->dynamicsWorld_->updateAabbs();

        sceneManager_->dynamicsWorld_->updateSingleAabb(body_);

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
        sceneManager_->dynamicsWorld_->contactTest(body_,contactCallback);

        lastTick_ = tick;
    }

    std::cout << "done callback for " << stringify(osg::Vec3(x,y,z)) << " collisionOffset: " << stringify(collisionOffset_) << std::endl;
    */

 //bool isInWorld = body_->isInWorld();
 //   if ( isInWorld ) sceneManager_->dynamicsWorld_->removeRigidBody(body_);

    btTransform tmpTransform = currentTransform_;
    // std::cout << getID() << " transform A = " << stringify(asOsgVec3(currentTransform_.getOrigin())) << std::endl;
    tmpTransform.setOrigin(btVector3(x,y,z));

    if ( isDynamic_ && checkCollisions(tmpTransform) ) {
        ShapeNode::setTranslation(x+collisionOffset_.x(), y+collisionOffset_.y(), z+collisionOffset_.z());
        //ShapeNode::setTranslation(collisionOffset_.x(), collisionOffset_.y(), collisionOffset_.z());
        //std::cout << getID() << "setTranslation offset " << stringify(collisionOffset_) << std::endl;
    } else {
        ShapeNode::setTranslation(x, y, z);
    }

    currentTransform_.setOrigin(asBtVector3(this->getTranslation()));
    body_->getMotionState()->setWorldTransform(currentTransform_);
    body_->setWorldTransform(currentTransform_);   
    if ( isDynamic_ ) {
        body_->activate();
        sceneManager_->dynamicsWorld_->updateSingleAabb(body_);
    }
    //if ( isInWorld )  sceneManager_->dynamicsWorld_->addRigidBody(body_);


    /*  {
        btScalar m[16];
        btDefaultMotionState* myMotionState = (btDefaultMotionState*) body_->getMotionState();
        myMotionState->m_graphicsWorldTrans.getOpenGLMatrix(m);
        osg::Matrixf mat(m);
        std::cout << getID(); printf("...setTranslation %f %f %f\n", mat.getTrans().x(),mat.getTrans().y(),mat.getTrans().z() );
    }


    btDefaultMotionState* myMotionState = (btDefaultMotionState*) body_->getMotionState();
    myMotionState->getWorldTransform( currentTransform_ );
    std::cout << getID() << " transform B = " << stringify(asOsgVec3( currentTransform_.getOrigin())) << std::endl;
    */

    ///// [Q] should we stop all velocities?



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
    if ( isDynamic_ && checkCollisions(tmpTransform) ) {
        ShapeNode::translate(collisionOffset_.x(), collisionOffset_.y(), collisionOffset_.z());
    }

    ShapeNode::setOrientationQuat(x, y, z, w);
    btQuaternion quat = asBtQuaternion(this->getOrientationQuat());
    currentTransform_.setRotation(quat);
    body_->getMotionState()->setWorldTransform(currentTransform_);
    body_->setWorldTransform(currentTransform_);
    if ( isDynamic_ ) body_->activate();
}

void CollisionShape::setOrientation(float pitch, float roll, float yaw)
{
    osg::Quat newQ = osg::Quat( osg::DegreesToRadians(pitch), osg::Vec3d(1,0,0),
                                osg::DegreesToRadians(roll), osg::Vec3d(0,1,0),
                                osg::DegreesToRadians(yaw), osg::Vec3d(0,0,1));
    btTransform tmpTransform = currentTransform_;
    tmpTransform.setRotation(asBtQuaternion(newQ));
    if ( isDynamic_ && checkCollisions(tmpTransform) ) {
        ShapeNode::translate(collisionOffset_.x(), collisionOffset_.y(), collisionOffset_.z());
    }




    ShapeNode::setOrientation(pitch, roll, yaw);

    // TODO: use global matrix

    btQuaternion quat = asBtQuaternion(this->getOrientationQuat());
    currentTransform_.setRotation(quat);

    body_->getMotionState()->setWorldTransform(currentTransform_);
    body_->setWorldTransform(currentTransform_);
    if ( isDynamic_ ) body_->activate();
}

void CollisionShape::setScale(float x, float y, float z)
{
    ShapeNode::setScale(x,y,z);


    // TODO: use global matrix


    //currentTransform_ = asBtTransform(mainTransform->getMatrix());
    //body_->getMotionState()->setWorldTransform(currentTransform_);
    if ( shape != ShapeNode::PLANE )
        collisionShp_->setLocalScaling(asBtVector3(this->getScale()));
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
/*void CollisionShape::drawShape()
{   printf("CollisionShape::drawShape()\n");
    ShapeNode::drawShape();
    
    // update the collision object's shape:

}
*/
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

    msg = lo_message_new();
    lo_message_add(msg, "sf", "setBounciness", getBounciness());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "sf", "setFriction", getFriction());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "sf", "setWindFactor", getWindFactor());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "sf", "setRollingFriction", getRollingFriction());
    ret.push_back(msg);
    
    msg = lo_message_new();
    lo_message_add(msg, "si", "setReportContacts", getReportContacts());
    ret.push_back(msg);

    return ret;
}

} // end of namespace spin

