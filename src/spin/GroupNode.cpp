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

#include <osg/ComputeBoundsVisitor>
#include <osgGA/GUIEventAdapter>
#include <osg/BlendFunc>
#include <osg/BlendColor>

#include <osgManipulator/TabBoxDragger>
#include <osgManipulator/TabBoxTrackballDragger>
#include <osgManipulator/TabPlaneDragger>
#include <osgManipulator/TabPlaneTrackballDragger>
//#include <osgManipulator/TrackballDragger>
#include <osgManipulator/Translate1DDragger>
#include <osgManipulator/Translate2DDragger>
//#include <osgManipulator/TranslateAxisDragger>
#include "DraggerWith3Axes.h"
#include "DraggerTrackball.h"

#include "GroupNode.h"
#include "SceneManager.h"
#include "spinApp.h"
#include "spinBaseContext.h"
#include "osgUtil.h"
#include "UserNode.h"
#include "spinApp.h"

extern pthread_mutex_t sceneMutex;

namespace spin
{

DraggerCallback::DraggerCallback(GroupNode* g) : DraggerTransformCallback(g->getTransform())
{
    groupNode = g;
}

bool DraggerCallback::receive(const osgManipulator::MotionCommand& command)
{
    if (!_transform) return false;
    
    bool success = false;
        
    switch (command.getStage())
    {
        case osgManipulator::MotionCommand::START:
        {
            //std::cout << "DraggerCallback START " << std::endl;
            
            // Save the current matrix
            _startMotionMatrix = _transform->getMatrix();
            
            // Get the LocalToWorld and WorldToLocal matrix for this node.
            osg::NodePath nodePathToRoot;
            osgManipulator::computeNodePathToRoot(*_transform,nodePathToRoot);
            _localToWorld = osg::computeLocalToWorld(nodePathToRoot);
            _worldToLocal = osg::Matrix::inverse(_localToWorld);
            
            success = true;
            break;
        }
        case osgManipulator::MotionCommand::MOVE:
        {
            osg::Matrix m = _localToWorld * command.getWorldToLocal()
                              * command.getMotionMatrix()
                              * command.getLocalToWorld()
                              * _worldToLocal
                              * _startMotionMatrix;
            
            /*
            std::cout << "MotionMatrix:" << std::endl;
            osg::Matrix mm = command.getMotionMatrix();
            std::cout << "  t  = " << stringify(mm.getTrans()) << std::endl;
            std::cout << "  q  = " << stringify(mm.getRotate()) << std::endl;
            std::cout << "  s  = " << stringify(mm.getScale()) << std::endl;
            
            std::cout << "DraggerCallback MOVE: " << std::endl;
            std::cout << "  t  = " << stringify(m.getTrans()) << std::endl;
            std::cout << "  q  = " << stringify(m.getRotate()) << std::endl;
            std::cout << "  s  = " << stringify(m.getScale()) << std::endl;
            */
        
                
            // Note: we don't know if the DraggerCallback is called on a client
            // machine or a server. For example, the event might originate with
            // mouse events in ViewerManipulator.
            
            // Each requires a different form of sending the update:

            
            
            // Use matrix::decompose and never matrix::getRotate when dealing
            // with scaled matrices:
            osg::Vec3 t;
            osg::Quat q;
            osg::Vec3 s;
            osg::Quat so;
            m.decompose(t, q, s, so);
            
            if (spinApp::Instance().getContext()->isServer())
            {
                groupNode->setTranslation(t.x(), t.y(), t.z());
                groupNode->setOrientationQuat(q.x(), q.y(), q.z(), q.w());
                groupNode->setScale(s.x(), s.y(), s.z());
            }
            else
            {
                spinApp::Instance().NodeMessage(groupNode->getID().c_str(), "sfff", "setTranslation", t.x(), t.y(), t.z(), SPIN_ARGS_END);
                spinApp::Instance().NodeMessage(groupNode->getID().c_str(), "sffff", "setOrientationQuat", q.x(), q.y(), q.z(), q.w(), SPIN_ARGS_END);
                spinApp::Instance().NodeMessage(groupNode->getID().c_str(), "sfff", "setScale", s.x(), s.y(), s.z(), SPIN_ARGS_END);
            }
            
            
            /*
            if (spinApp::Instance().getContext()->isServer())
            {
                groupNode->setManipulatorMatrix(
                    m(0,0),m(0,1),m(0,2),m(0,3),
                    m(1,0),m(1,1),m(1,2),m(1,3),
                    m(2,0),m(2,1),m(2,2),m(2,3),
                    m(3,0),m(3,1),m(3,2),m(3,3));
            }
            else
            {
                spinApp::Instance().NodeMessage(groupNode->getID().c_str(), "sffffffffffffffff", "setManipulatorMatrix",
                    m(0,0),m(0,1),m(0,2),m(0,3),
                    m(1,0),m(1,1),m(1,2),m(1,3),
                    m(2,0),m(2,1),m(2,2),m(2,3),
                    m(3,0),m(3,1),m(3,2),m(3,3),
                    SPIN_ARGS_END);
            }
            */
            
            success = true;
            break;
        }
        case osgManipulator::MotionCommand::FINISH:
        {
            groupNode->updateDraggerMatrix();
            success = true;
            break;
        }
        case osgManipulator::MotionCommand::NONE:
        default:
            success = false;
    }

    return success;
}
    
    
// ***********************************************************
// constructor:
GroupNode::GroupNode (SceneManager *sceneManager, const char* initID) : ReferencedNode(sceneManager, initID)
{
    this->setNodeType("GroupNode");
    this->setName(getID() + ".GroupNode");

    //_reportGlobals = false;
    reportMode_ = GroupNode::NONE;
    interactionMode_ = GroupNode::STATIC;
    orientationMode_ = GroupNode::NORMAL;
    
    orientationTarget_ = gensym("NULL");
    manipulatorType_ = "NULL";
    manipulatorUpdateFlag_ = false;
    manipulatorShadowCopy_ = true;

    mainTransform_ = new osg::MatrixTransform();
    mainTransform_->setName(getID() + ".MainTransform");
    //mainTransform_->setAttitude(osg::Quat(0.0,0.0,0.0,0.0));
    //mainTransform_->setPosition(osg::Vec3(0.0,0.0,0.0));
    this->addChild(mainTransform_.get());

    computationMode_ = SERVER_SIDE;
    
    maxUpdateDelta_ = 0.05; // update when dt is at least 0.05s (ie 20hz)
    //maxUpdateDelta_ = 0.1; // update when dt is at least 0.1s (ie 10hz)

    stateset_ = gensym("NULL");

    clipNode_ = new osg::ClipNode();
    clipNode_->setCullingActive(false);
    clipNode_->setName(getID() + ".ClipNode");
    mainTransform_->addChild(clipNode_.get());
    
    scale_ = osg::Vec3(1.0,1.0,1.0);
    velocity_ = osg::Vec3(0.0,0.0,0.0);
    velocityMode_ = GroupNode::TRANSLATE;
    spin_ = osg::Vec3(0.0,0.0,0.0);
    damping_ = 0.0;
    
    broadcastLock_ = false;
    
    updateMatrix();

    // When children are attached to this, they get added to the attachNode:
    // NOTE: by changing this, we MUST override the updateNodePath() method!
    setAttachmentNode(clipNode_.get());

    // keep a timer for velocity calculation:
    lastTick_ = osg::Timer::instance()->tick();
    lastUpdate_ = osg::Timer::instance()->tick();
}

// ***********************************************************
// destructor
GroupNode::~GroupNode()
{

}

#define EPSILON 0.0001

void GroupNode::callbackUpdate(osg::NodeVisitor* nv)
{
    ReferencedNode::callbackUpdate(nv);
    
    if (manipulatorUpdateFlag_)
    {
        drawManipulator();
        manipulatorUpdateFlag_ = false;
    }
    
    osg::Timer_t tick = osg::Timer::instance()->tick();
    float dt = osg::Timer::instance()->delta_s(lastUpdate_,tick);

    if (dt > maxUpdateDelta_)
        dumpGlobals(false); // never force globals here
    
    // Decide whether messages should be broadcasted during this update:
    if (spinApp::Instance().getContext()->isServer())
    {
        // if this node does computation on the client side, don't let the
        // server broadcast any messages:
        if (computationMode_==CLIENT_SIDE)
        {
            broadcastLock_ = true;
        }
        
        // on the server-side, we want to throttle network messages, so only
        // allow broadcasting of messages if a threshold amount of time has
        // passed:
        else if (dt > maxUpdateDelta_)
        {
            broadcastLock_ = false;
            lastUpdate_ = tick;
        }
        else 
        {
            broadcastLock_ = true;
        }
    }
    
     
    // We update state in the following situations:
    // 1) this is a server (always needs to be up-to-date for interaction)
    // 2) this is a client and the node is set to client-side computation.
    if (spinApp::Instance().getContext()->isServer() || (!spinApp::Instance().getContext()->isServer() && (computationMode_==CLIENT_SIDE)))
    {
	// now dt is the time since the "lastTick" (could be more recent than the
        // last update
    	dt = osg::Timer::instance()->delta_s(lastTick_,tick);

        // Now we need to update translation/orientation based on the current
        // velocity/spin values. We find out how many seconds passed since the
        // last time this was called, and move by velocity_*dt (ie, m/s) and 
        // rotate by spin_*dt (ie, deg/sec)
        if (motion_.valid())
        {
            motion_->update(dt);
            float motionIndex = motion_->getValue();
            osg::Vec3 newPos = motionStart_ + ((motionEnd_-motionStart_) * motionIndex);
            this->setTranslation( newPos.x(), newPos.y(), newPos.z() );
            
            if (motion_->getTime() >= motion_->getDuration())
            {
                BROADCAST(this, "sss", "event", "translateTo", "done");
                motion_ = NULL;
            }
        }


        if (velocity_.length() > EPSILON)
        {
            if (this->velocityMode_==GroupNode::TRANSLATE)
            {
                this->translate( velocity_.x()*dt, velocity_.y()*dt, velocity_.z()*dt );
            }
            else
            {
                this->move( velocity_.x()*dt, velocity_.y()*dt, velocity_.z()*dt );
            }
            if (damping_ > EPSILON)
            {
                double dv = 1 - (damping_*dt);
                this->setVelocity( velocity_.x()*dv, velocity_.y()*dv, velocity_.z()*dv );
            }
        }
        else velocity_ = osg::Vec3(0,0,0);

        if (spin_.length() > EPSILON)
        {
            if (this->velocityMode_==GroupNode::TRANSLATE)
            	this->rotate( spin_.x()*dt, spin_.y()*dt, spin_.z()*dt );
            else
            	this->addRotation( spin_.x()*dt, spin_.y()*dt, spin_.z()*dt );

            if (damping_ > EPSILON)
            {
                double ds = 1 - (damping_*dt);
                this->setSpin( spin_.x()*ds, spin_.y()*ds, spin_.z()*ds );
            }
        }
        else spin_ = osg::Vec3(0,0,0);
    }
    
    lastTick_ = tick;
    broadcastLock_ = false;
}

void GroupNode::updateNodePath(bool updateChildren)
{
    // get node path from parent class, but with the false flag so that children
    // are not updated yet:
	ReferencedNode::updateNodePath(false);

    currentNodePath_.push_back(mainTransform_.get());    
    currentNodePath_.push_back(clipNode_.get());
    
    // now update NodePaths for all children:
    if (updateChildren) updateChildNodePaths();
}

// *****************************************************************************

void GroupNode::mouseEvent (int /*event*/, int keyMask, int buttonMask, float x, float y)
{
    // TODO
    BROADCAST(this, "siiiff", "mouseEvent", keyMask, buttonMask, x, y);
}

void GroupNode::event (int event, const char* userString, float eData1, float eData2, float x, float y, float z)
{
    osg::ref_ptr<UserNode> user = dynamic_cast<UserNode*>(sceneManager_->getNode(userString));
    if (!user.valid()) return;

    if (interactionMode_==DRAG || interactionMode_==THROW)
    {
        switch(event)
        {
            case(osgGA::GUIEventAdapter::PUSH):

                if (!this->owner_.valid())
                {
                    this->setVelocity(0,0,0);
                    this->setSpin(0,0,0);
                    trajectory_.clear();
                }

                break;

            case(osgGA::GUIEventAdapter::RELEASE):

                if (this->owner_ == user)
                {
                    if (interactionMode_==THROW)
                    {
                        // Take average of stored motion vectors, and set velocity in
                        // that direction. Note: damping_ should be set so that node
                        // gradually stops:

                        double currentTime = osg::Timer::instance()->time_m();

                        osg::Vec3 vel = osg::Vec3(0,0,0);
                        int count = 0;

                        std::vector<osg::Vec4>::iterator it;
                        for (it=trajectory_.begin(); it!=trajectory_.end(); ++it)
                        {
                            if (currentTime - (*it).w() < 500)
                            {
                                vel += osg::Vec3((*it).x(),(*it).y(),(*it).z());
                                count++;
                            }
                        }

                        if (count)
                        {
                            vel /= count;
                            vel *= 10; // motion vectors were typically small, so scale

                            this->setVelocity(vel.x(), vel.y(), vel.z());
                        }
                    }
                }
                break;

            case(osgGA::GUIEventAdapter::DRAG):

                if (this->owner_ == user)
                {

                    // if this node is owned by the event's user, then we apply the
                    // motion relative to the user's current position/orientation:

                    const osg::Matrix targMatrix = this->getGlobalMatrix();
                    const osg::Matrix userMatrix = user->getGlobalMatrix();

                    float distance = (targMatrix.getTrans() - userMatrix.getTrans()).length();

                    // perspective projection:
                    //float f = 29.1489;
                    float f = 3.9;  // why this number?
                    float dx = distance * -eData1 / f;
                    float dy = distance * -eData2 / f;

                    osg::Vec3 motionVec = userMatrix.getRotate() * osg::Vec3(dx,0,dy);
                    osg::Vec3 newPos = getTranslation() + motionVec;
                    this->setTranslation(newPos.x(), newPos.y(), newPos.z());

                    // save last N motion vectors, so we can setVelocity on RELEASE:
                    // (note, we use a vec4, and store a time in ms as the 4th value
                    trajectory_.insert(trajectory_.begin(), osg::Vec4(motionVec, osg::Timer::instance()->time_m()));
                    if (trajectory_.size() > 5) {
                        trajectory_.pop_back();
                    }
                }

                break;
        }

        // broadcast event
        // NO! can't do this, because we will duplicate move/rotate/etc actions

    }

    else if (interactionMode_==DRAW)
    {
        if ((event==osgGA::GUIEventAdapter::PUSH) || (event==osgGA::GUIEventAdapter::RELEASE))
        {
            if ((int)eData2==osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) drawMod_ = 1;
            else if ((int)eData2==osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON) drawMod_ = 2;
            else drawMod_ = 0;

            BROADCAST(this, "ssifff", "draw", userString, drawMod_, x, y, z);
        }
        else if (event==osgGA::GUIEventAdapter::DRAG)
        {
            BROADCAST(this, "ssifff", "draw", userString, drawMod_, x, y, z);
        }
    }

    // after all is done, we update ownership (if this was a PUSH or RELEASE)
    if (interactionMode_==SELECT || interactionMode_==DRAG || interactionMode_==THROW)
    {
        switch(event)
        {
            case(osgGA::GUIEventAdapter::PUSH):

                // on push, we check if the current node is owned by anyone,
                // and if not, we give ownnerhip to the event's user:
                if (!this->owner_.valid())
                {
                    this->owner_ = user.get();
                    //std::cout << "setting owner of " << getID() << " to " << owner_->getID() << std::endl;
                    BROADCAST(this, "ss", "setOwner", owner_->getID().c_str());
                    BROADCAST(this, "ssi", "select", userString, 1);
                }
                break;

            case(osgGA::GUIEventAdapter::RELEASE):

                // on release, the user gives up ownership of the node (assuming
                // that he had ownership in the first place):
                if (this->owner_ == user)
                {
                    //std::cout << "setting owner of " << getID() << " to NULL" << std::endl;
                    this->owner_ = NULL;
                    BROADCAST(this, "ss", "setOwner", "NULL");
                    BROADCAST(this, "ssi", "select", userString, 0);
               }
                break;
            case(osgGA::GUIEventAdapter::DOUBLECLICK):
                BROADCAST(this, "ss", "doubleclick", userString);
                break;
        }

    }
}

void GroupNode::setLock(const char* userString, int lock)
{
    osg::ref_ptr<UserNode> user = dynamic_cast<UserNode*>(sceneManager_->getNode(userString));
    if (!user.valid()) return;

    if (lock)
    {
        if (owner_.valid())
        {
            this->owner_ = user.get();
            //std::cout << "setting owner of " << getID() << " to " << owner_->getID() << std::endl;
            BROADCAST(this, "ss", "setOwner", owner_->getID().c_str());
        }
    }
    else
    {
        if (this->owner_ == user)
        {
            //std::cout << "setting owner of " << getID() << " to NULL" << std::endl;
            this->owner_ = NULL;
            BROADCAST(this, "ss", "setOwner", "NULL");
        }
    }
}

void GroupNode::debug()
{
    ReferencedNode::debug();

    std::cout << "   Manipulator: " << manipulatorType_ << std::endl;
    if (owner_.valid()) std::cout << "   owner: " << owner_->getID() << std::endl;
    else std::cout << "   owner: NULL" << std::endl;
    
    std::cout << "   mainX posn: " << stringify(this->getTranslation()) << std::endl;
    std::cout << "   mainX qua: " << stringify(this->getOrientationQuat()) << std::endl;
    std::cout << "   mainX ori: " << stringify(this->getOrientation()) << std::endl;
    std::cout << "   mainX scl: " << stringify(this->getScale()) << std::endl;
    
    const osg::Matrix tmpMatrix = this->getGlobalMatrix();
    osg::Vec3 t;
    osg::Quat q;
    osg::Vec3 s;
    osg::Quat so;
    tmpMatrix.decompose(t, q, s, so);
    std::cout << "   global pos: " << stringify(t) << std::endl;
    std::cout << "   global qua: " << stringify(q) << std::endl;
    std::cout << "   global ori: " << stringify(Vec3inDegrees(QuatToEuler(q))) << std::endl;
    std::cout << "   global scl: " << stringify(s) << std::endl;

}


// -----------------------------------------------------------------------------

void GroupNode::setUpdateRate(float seconds)
{
    maxUpdateDelta_ = seconds;
    BROADCAST(this, "sf", "setUpdateRate", getUpdateRate());
}


void GroupNode::setComputationMode (ComputationMode mode)
{

    if (this->computationMode_ != mode)
    {
        this->computationMode_ = mode;
        lastUpdate_ = lastTick_;
        BROADCAST(this, "si", "setComputationMode", getComputationMode());
    }
}

void GroupNode::setReportMode (globalsReportMode mode)
{
    if (this->reportMode_ != mode)
    {
        this->reportMode_ = mode;
        BROADCAST(this, "si", "setReportMode", getReportMode());
    }
}

void GroupNode::setInteractionMode (InteractionMode mode)
{
    if (this->interactionMode_ != mode)
    {

        // if this node previously had an owner, it will be lost with the change
        // of mode
        if (this->owner_.valid())
        {
            this->owner_ = NULL;
            BROADCAST(this, "ss", "owner", "NULL");
        } else if (interactionMode_ == DRAW)
        {
            BROADCAST(this, "ssifff", "draw", "NULL", 0, 0.0f, 0.0f, 0.0f);
        }

        this->interactionMode_ = mode;



        // set the node mask so that an intersection visitor will only test
        // for interactive nodes (note: nodemask info in spinUtil.h)
        if ((int)interactionMode_ > 0)
            this->setNodeMask(INTERACTIVE_NODE_MASK);
        else
            this->setNodeMask(GEOMETRIC_NODE_MASK);

        BROADCAST(this, "si", "setInteractionMode", getInteractionMode());
        
        // now we need to travel up the scenegraph and set all parents to have
        // an InteractionMode of PASSTHRU, otherwise an intersection traversal
        // will never find this node:
                
        GroupNode *parent = dynamic_cast<GroupNode*>(this->getParentNode(0));
        while (parent)
        {
            if (!parent->getInteractionMode())
            {
                parent->setInteractionMode(PASSTHRU);
                continue; // rest of parents will be recursively called
            }
            else parent = dynamic_cast<GroupNode*>(parent->getParentNode(0));
        }
        
    }
}

// -----------------------------------------------------------------------------

void GroupNode::setStateSetFromFile(const char* filename)
{
	osg::ref_ptr<ReferencedStateSet> ss = sceneManager_->createStateSet(filename);
	if (ss.valid())
	{
		if (ss->getIDSymbol() == stateset_) return; // we're already using that stateset
		stateset_ = ss->getIDSymbol();
		updateStateSet();
		BROADCAST(this, "ss", "setStateSet", getStateSetSymbol()->s_name);
	}
}

void GroupNode::setStateSet (const char* s)
{
	if (gensym(s)==stateset_) return;

	osg::ref_ptr<ReferencedStateSet> ss = sceneManager_->getStateSet(s);
	if (ss.valid())
	{
		stateset_ = ss->getIDSymbol();
		updateStateSet();
		BROADCAST(this, "ss", "setStateSet", getStateSetSymbol()->s_name);
	}
}

void GroupNode::updateStateSet()
{
	osg::ref_ptr<ReferencedStateSet> ss = dynamic_cast<ReferencedStateSet*>(stateset_->s_thing);
	if (mainTransform_.valid() && ss.valid()) mainTransform_->setStateSet( ss.get() );
}



// -----------------------------------------------------------------------------

void GroupNode::setClipping(float x, float y, float z)
{
    clipping_ = osg::Vec3d(x,y,z);

    // remove existing clipping planes:
    while (clipNode_->getNumClipPlanes()) clipNode_->removeClipPlane((unsigned int)0);

    // add this one:
    if ( (clipping_.x()>0) && (clipping_.y()>0) && (clipping_.z()>0) )
    {
        osg::BoundingBox bb(-clipping_.x(),-clipping_.y(),-clipping_.z(),clipping_.x(),clipping_.y(),clipping_.z());
        clipNode_->createClipBox(bb);
    }

    BROADCAST(this, "sfff", "setClipping", clipping_.x(), clipping_.y(), clipping_.z());
}

void GroupNode::updateQuat()
{
    quat_ = osg::Quat( osg::DegreesToRadians(orientation_.x()), osg::Vec3d(1,0,0),
                       osg::DegreesToRadians(orientation_.y()), osg::Vec3d(0,1,0), 
                       osg::DegreesToRadians(orientation_.z()), osg::Vec3d(0,0,1));
            
    // fix numerical imprecision:
    float epsilon = 0.0000001;
    if (fabs(quat_.x())<epsilon) quat_ = osg::Quat(0.0,quat_.y(),quat_.z(),quat_.w());
    if (fabs(quat_.y())<epsilon) quat_ = osg::Quat(quat_.x(),0.0,quat_.z(),quat_.w());
    if (fabs(quat_.z())<epsilon) quat_ = osg::Quat(quat_.x(),quat_.y(),0.0,quat_.w());
    if (fabs(quat_.w())<epsilon) quat_ = osg::Quat(quat_.x(),quat_.y(),quat_.z(),0.0);

    updateMatrix();
}

void GroupNode::updateMatrix()
{
    osg::Matrix matrix;
    /*
    // from osg::PositionAttitudeTransform (RELATIVE_RF)
    matrix.preMultTranslate(translation_);
    matrix.preMultRotate(quat_);
    matrix.preMultScale(scale_);
    */
    
    // from osg::PositionAttitudeTransform (ABSOLUTE)
    matrix.makeRotate(quat_);
    matrix.postMultTranslate(translation_);
    matrix.preMultScale(scale_);
    
    /*
    matrix = osg::Matrix::translate(translation_)
            * osg::Matrix::scale(scale_)
            * osg::Matrix::rotate(quat_);
    */
    mainTransform_->setMatrix(matrix);
 
}

void GroupNode::updateDraggerMatrix()
{
    osg::Matrix matrix;
    matrix.preMultTranslate(translation_);
    matrix.preMultScale(dragger_->getMatrix().getScale());
    dragger_->setMatrix(matrix);
}

void GroupNode::setTranslation (float x, float y, float z)
{
    osg::Vec3 newTranslation = osg::Vec3(x,y,z);

    if (newTranslation != translation_)
    {
        translation_ = newTranslation;
        updateMatrix();

        //if (dragger_.valid()) updateDraggerMatrix();
        
        if (!broadcastLock_)
            BROADCAST(this, "sfff", "setTranslation", x, y, z);
    }
    
    // if we've moved and the orientationMode requires us to point to a target
    // node or the origin, we should apply that orientation change too:
    if (orientationMode_!=NORMAL)
    {
        applyOrientationMode();
    }
}

void GroupNode::setOrientationQuat (float x, float y, float z, float w)
{
    if ((orientationMode_==NORMAL)||(!spinApp::Instance().getContext()->isServer()))
    {
        osg::Quat newQuat = osg::Quat(x,y,z,w);

        if (newQuat != quat_)
        {
            quat_ = newQuat;
            updateMatrix();
            orientation_ = Vec3inDegrees(QuatToEuler(newQuat));

            if (!broadcastLock_)
                BROADCAST(this, "sffff", "setOrientationQuat", x, y, z, w);
        }
    }
}

void GroupNode::setOrientation (float p, float r, float y)
{
    // Only allow orientation changes if OrientationMode is NORMAL, or if this
    // is a client. The server cannot apply orientations for other modes, but
    // clients do need to be notified if the server sends a constrained 
    // orientation
    if ((orientationMode_==NORMAL)||(!spinApp::Instance().getContext()->isServer()))
    {
        osg::Vec3 newOrientation = osg::Vec3(p, r, y);

        if (newOrientation != orientation_)
        {
            orientation_ = newOrientation;
            updateQuat();
            if (!broadcastLock_)
                BROADCAST(this, "sfff", "setOrientation", p, r, y);
        }
    }
}

void GroupNode::setOrientationMode (OrientationMode m)
{
	this->orientationMode_ = m;
	BROADCAST(this, "si", "setOrientationMode", (int)orientationMode_);
    if (orientationMode_!=NORMAL)
    {
        applyOrientationMode();
    }
}

void GroupNode::setOrientationTarget (const char* target)
{
    orientationTarget_ = gensym(target);
    if (orientationMode_!=NORMAL)
    {
        applyOrientationMode();
    }
    BROADCAST(this, "ss", "setOrientationTarget", getOrientationTarget());
}

void GroupNode::applyOrientationMode()
{
    if (spinApp::Instance().getContext()->isServer() || (!spinApp::Instance().getContext()->isServer() && (computationMode_==CLIENT_SIDE)))
    {
    
        osg::Vec3 targetPos;
        osg::Matrix thisMatrix = osg::computeLocalToWorld(this->currentNodePath_);
        
        if ((orientationMode_==POINT_TO_TARGET)||(orientationMode_==POINT_TO_TARGET_CENTROID))
        {
            ReferencedNode *target = dynamic_cast<ReferencedNode*>(orientationTarget_->s_thing);
            if (target)
            {
                // TODO: verify if we really need to recompute the matrices and bound every time.
                // Perhaps the matrix is already stored somewhere in certain cases?
                
                if (orientationMode_==POINT_TO_TARGET_CENTROID)
                {
                    targetPos = target->computeBound().center();
                }
                else
                {
                    osg::Matrixd targetMatrix = osg::computeLocalToWorld(target->currentNodePath_);
                    targetPos = targetMatrix.getTrans();
                }
            }
        }
        else if (orientationMode_==POINT_TO_ORIGIN)
        {
            targetPos = osg::Vec3(0.0,0.0,0.0);
        }
        
        osg::Vec3 aed = cartesianToSpherical(targetPos - thisMatrix.getTrans());
        
        // can't do this, because that function only does something for NORMAL mode
        //setOrientation(osg::RadiansToDegrees(aed.y()), 0.0, osg::RadiansToDegrees(aed.x()));

        orientation_ = osg::Vec3(osg::RadiansToDegrees(aed.y()), 0.0, osg::RadiansToDegrees(aed.x()));
        quat_ = osg::Quat( osg::DegreesToRadians(orientation_.x()), osg::Vec3d(1,0,0),
                                 osg::DegreesToRadians(orientation_.y()), osg::Vec3d(0,1,0),
                                 osg::DegreesToRadians(orientation_.z()), osg::Vec3d(0,0,1));
        updateMatrix();
    }
    
    if (!broadcastLock_)
        BROADCAST(this, "sfff", "setOrientation", orientation_.x(), orientation_.y(),  orientation_.z());
}


/*
void GroupNode::setOrientationQuat (float x, float y, float z, float w)
{
    osg::Quat = osg::Quat(x,y,z,w);
    orientation_ = QuatToEuler(q);
    mainTransform_->setAttitude(q);

    BROADCAST(this, "sfff", "setOrientation", _orientatoin.x(), orientation_.y(), orientation_.z());
}
*/

void GroupNode::setScale (float x, float y, float z)
{
    osg::Vec3 newScale = osg::Vec3(x,y,z);

    if (newScale != scale_)
    {
        scale_ = newScale;
        updateMatrix();
        
        if (!broadcastLock_)
            BROADCAST(this, "sfff", "setScale", x, y, z);
    }
}

void GroupNode::setVelocity (float dx, float dy, float dz)
{
    osg::Vec3 newVelocity = osg::Vec3(dx,dy,dz);

    if (newVelocity != velocity_)
    {
        velocity_ = newVelocity;
        if (!broadcastLock_)
            BROADCAST(this, "sfff", "setVelocity", dx, dy, dz);
            
        // if this is an object that was updated on the client side, and the
        // velocity is stopped, let's do a server-side push of the latest state
        // to ensure the client state is perfectly synchronized:
        if ((velocity_.length()==0) && (computationMode_==CLIENT_SIDE))
        {
            BROADCAST(this, "sfff", "setTranslation", translation_.x(), translation_.y(), translation_.z());
            BROADCAST(this, "sfff", "setOrientation", orientation_.x(), orientation_.y(), orientation_.z());
        }
    }
}

void GroupNode::setVelocityMode (velocityMode mode)
{
    if (this->velocityMode_ != (int)mode)
    {
        this->velocityMode_ = mode;
        BROADCAST(this, "si", "setVelocityMode", (int) this->velocityMode_);
    }
}

void GroupNode::setSpin (float dp, float dr, float dy)
{
    osg::Vec3 newSpin = osg::Vec3(dp,dr,dy);

    if (newSpin != spin_)
    {
        spin_ = newSpin;
        if (!broadcastLock_)
            BROADCAST(this, "sfff", "setSpin", dp, dr, dy);
            
        // if this is an object that was updated on the client side, and the
        // spin is stopped, let's do a server-side push of the latest state
        // to ensure the client state is perfectly synchronized:
        if ((spin_.length()==0) && (computationMode_==CLIENT_SIDE))
        {
            BROADCAST(this, "sfff", "setTranslation", translation_.x(), translation_.y(), translation_.z());
            BROADCAST(this, "sfff", "setOrientation", orientation_.x(), orientation_.y(), orientation_.z());
        }

    }
}

void GroupNode::setDamping (float d)
{
    if (d != damping_)
    {
        damping_ = d;
        BROADCAST(this, "sf", "setDamping", damping_);
    }
}

void GroupNode::translate (float x, float y, float z)
{
    // simple move relative to the parent
    osg::Vec3 newPos = getTranslation() + osg::Vec3(x,y,z);
    setTranslation(newPos.x(), newPos.y(), newPos.z());
}

void GroupNode::move (float x, float y, float z)
{
    // take the orientation into account, and move along that vector:
    osg::Vec3 newPos = getTranslation() + ( getOrientationQuat() * osg::Vec3(x,y,z) );
    setTranslation(newPos.x(), newPos.y(), newPos.z());
}

void GroupNode::rotate (float dPitch, float dRoll, float dYaw)
{
    this->setOrientation(orientation_.x()+dPitch, orientation_.y()+dRoll, orientation_.z()+dYaw);
}

void GroupNode::addRotation (float dPitch, float dRoll, float dYaw)
{
    osg::Quat newQuat = EulerToQuat(dPitch,dRoll,dYaw).inverse() * getOrientationQuat();
    this->setOrientationQuat(newQuat.x(), newQuat.y(), newQuat.z(), newQuat.w());
}

// *****************************************************************************



void GroupNode::translateTo (float x, float y, float z, float duration, const char *motion)
{

    if (motion_.valid())
    {
        motion_ = NULL;
    }

    motionStart_ = getTranslation();
    motionEnd_ = osg::Vec3(x,y,z);
    
    std::string motionType = std::string(motion);
    
    osgAnimation::Motion::TimeBehaviour tb = osgAnimation::Motion::CLAMP;
    float init = 0.0f;
    float d = 1.0f;

    if (!motionType.compare("OutQuadMotion"))
        motion_ = new osgAnimation::OutQuadMotion(init, duration, d, tb);
    else if (!motionType.compare("InQuadMotion"))
        motion_ = new osgAnimation::InQuadMotion(init, duration, d, tb);
    else if (!motionType.compare("InOutQuadMotion"))
        motion_ = new osgAnimation::InOutQuadMotion(init, duration, d, tb);
    else if (!motionType.compare("OutCubicMotion"))
        motion_ = new osgAnimation::OutCubicMotion(init, duration, d, tb);
    else if (!motionType.compare("InCubicMotion"))
        motion_ = new osgAnimation::InCubicMotion(init, duration, d, tb);
    else if (!motionType.compare("InOutCubicMotion"))
        motion_ = new osgAnimation::InOutCubicMotion(init, duration, d, tb);
    else if (!motionType.compare("OutQuartMotion"))
        motion_ = new osgAnimation::OutQuartMotion(init, duration, d, tb);
    else if (!motionType.compare("InQuartMotion"))
        motion_ = new osgAnimation::InQuartMotion(init, duration, d, tb);
    else if (!motionType.compare("InOutQuartMotion"))
        motion_ = new osgAnimation::InOutQuartMotion(init, duration, d, tb);
    else if (!motionType.compare("OutBounceMotion"))
        motion_ = new osgAnimation::OutBounceMotion(init, duration, d, tb);
    else if (!motionType.compare("InBounceMotion"))
        motion_ = new osgAnimation::InBounceMotion(init, duration, d, tb);
    else if (!motionType.compare("InOutBounceMotion"))
        motion_ = new osgAnimation::InOutBounceMotion(init, duration, d, tb);
    else if (!motionType.compare("OutElasticMotion"))
        motion_ = new osgAnimation::OutElasticMotion(init, duration, d, tb);
    else if (!motionType.compare("InElasticMotion"))
        motion_ = new osgAnimation::InElasticMotion(init, duration, d, tb);
    else if (!motionType.compare("InOutElasticMotion"))
        motion_ = new osgAnimation::InOutElasticMotion(init, duration, d, tb);
    else if (!motionType.compare("OutSineMotion"))
        motion_ = new osgAnimation::OutSineMotion(init, duration, d, tb);
    else if (!motionType.compare("InSineMotion"))
        motion_ = new osgAnimation::InSineMotion(init, duration, d, tb);
    else if (!motionType.compare("InOutSineMotion"))
        motion_ = new osgAnimation::InOutSineMotion(init, duration, d, tb);
    else if (!motionType.compare("OutBackMotion"))
        motion_ = new osgAnimation::OutBackMotion(init, duration, d, tb);
    else if (!motionType.compare("InBackMotion"))
        motion_ = new osgAnimation::InBackMotion(init, duration, d, tb);
    else if (!motionType.compare("InOutBackMotion"))
        motion_ = new osgAnimation::InOutBackMotion(init, duration, d, tb);
    else if (!motionType.compare("OutCircMotion"))
        motion_ = new osgAnimation::OutCircMotion(init, duration, d, tb);
    else if (!motionType.compare("InCircMotion"))
        motion_ = new osgAnimation::InCircMotion(init, duration, d, tb);
    else if (!motionType.compare("InOutCircMotion"))
        motion_ = new osgAnimation::InOutCircMotion(init, duration, d, tb);
    else if (!motionType.compare("OutExpoMotion"))
        motion_ = new osgAnimation::OutExpoMotion(init, duration, d, tb);
    else if (!motionType.compare("InExpoMotion"))
        motion_ = new osgAnimation::InExpoMotion(init, duration, d, tb);
    else if (!motionType.compare("InOutExpoMotion"))
        motion_ = new osgAnimation::InOutExpoMotion(init, duration, d, tb);
    else
        motion_ = new osgAnimation::LinearMotion(init, duration, d, tb);

    if (computationMode_==CLIENT_SIDE)
    {
        BROADCAST(this, "sffffs", "translateTo", x, y, z, duration, motion);
    }
}

// *****************************************************************************

    
osgManipulator::Dragger* createDragger(const std::string& name, float size, osg::Vec3 offset, osg::Vec3 scale)
{
    osgManipulator::Dragger* dragger = 0;
    float scaleFactor = 1.0;
    if ("TabPlaneDragger" == name)
    {
        osgManipulator::TabPlaneDragger* d = new osgManipulator::TabPlaneDragger();
        d->setupDefaultGeometry();
        dragger = d;
        scaleFactor = 2.0;
    }
    else if ("TabPlaneTrackballDragger" == name)
    {
        osgManipulator::TabPlaneTrackballDragger* d = new osgManipulator::TabPlaneTrackballDragger();
        d->setupDefaultGeometry();
        dragger = d;
        scaleFactor = 2.5;
    }
    else if ("TabBoxTrackballDragger" == name)
    {
        osgManipulator::TabBoxTrackballDragger* d = new osgManipulator::TabBoxTrackballDragger();
        d->setupDefaultGeometry();
        dragger = d;
        scaleFactor = 2.5;
    }
    else if ("TrackballDragger" == name)
    {
        //osgManipulator::TrackballDragger* d = new osgManipulator::TrackballDragger();
        osgManipulator::DraggerTrackball* d = new osgManipulator::DraggerTrackball();
        d->setupDefaultGeometry();
        dragger = d;
        scaleFactor = 1.0;
    }
    else if ("Translate1DDragger" == name)
    {
        osgManipulator::Translate1DDragger* d = new osgManipulator::Translate1DDragger();
        d->setupDefaultGeometry();
        dragger = d;
        scaleFactor = 2.0;
    }
    else if ("Translate2DDragger" == name)
    {
        osgManipulator::Translate2DDragger* d = new osgManipulator::Translate2DDragger();
        d->setupDefaultGeometry();
        dragger = d;
        scaleFactor = 2.5;
    }
    else if ("TranslateAxisDragger" == name)
    {
        //osgManipulator::TranslateAxisDragger* d = new osgManipulator::TranslateAxisDragger();
        osgManipulator::DraggerWith3Axes* d = new osgManipulator::DraggerWith3Axes();
        d->setupDefaultGeometry();
        dragger = d;
        scaleFactor = 1.4;
    }
    else if ("TabBoxDragger" == name)
    {
        osgManipulator::TabBoxDragger* d = new osgManipulator::TabBoxDragger();
        d->setupDefaultGeometry();
        dragger = d;
        scaleFactor = 1.25; //1.6;
    }
    
    if (dragger)
    {
        dragger->setName(name);
        
        dragger->setMatrix(osg::Matrix::scale(size*scaleFactor, size*scaleFactor, size*scaleFactor) * osg::Matrix::translate(offset));
        //dragger->setMatrix(osg::Matrix::scale(scale.x()*scaleFactor, scale.y()*scaleFactor, scale.z()*scaleFactor) * osg::Matrix::translate(offset));
        
        // turn off lighing on dragger
        osg::StateSet *ss = dragger->getOrCreateStateSet();
        ss->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
    }
    
    return dragger;
}

void GroupNode::setManipulatorMatrix(float a00, float a01, float a02, float a03,
                                     float a10, float a11, float a12, float a13,
                                     float a20, float a21, float a22, float a23,
                                     float a30, float a31, float a32, float a33)
{
    
    osg::Matrix m = osg::Matrix(a00,a01,a02,a03,
                                a10,a11,a12,a13,
                                a20,a21,a22,a23,
                                a30,a31,a32,a33);

    mainTransform_->setMatrix(m);
    
    /*
    // Can't just broadcast the matrix, because it won't call the clients'
    // setTranslation, setOrientation and setScale method... thus, collisions
    // and any other constraints won't be handled (important if this instance
    // overrides those methods and does extra stuff).
    BROADCAST(this, "sffffffffffffffff", "setManipulatorMatrix",
                    a00,a01,a02,a03,
                    a10,a11,a12,a13,
                    a20,a21,a22,a23,
                    a30,a31,a32,a33);
    */
    
    
    // So we compute the independent translation, rotation, & scale components
    // from the matrix:
    
    osg::Vec3 t;
    osg::Quat q;
    osg::Vec3 s;
    
    // IMPORTANT: getRotate() doesn't take scale into account! watch out!
    /*
    t = m.getTrans();
    q = m.getRotate();
    s = m.getScale();
    */
    
    // Use matrix::decompose() instead (and note: so is 'scale orientation')
    osg::Quat so;
    m.decompose(t, q, s, so);

    /*
    std::cout << "decomp trans: " << stringify(t) << std::endl;
    std::cout << "decomp orien: " << stringify(q) << std::endl;
    std::cout << "decomp scale: " << stringify(s) << std::endl;
    std::cout << "decomp scOri: " << stringify(so) << std::endl;
    */


    // But, we can't call setTranslation etc here because those are assumed to
    // be manual method and cause the dragger matrix to be updated. ie, the 
    // updateDraggerMatrix() method...
    
    this->setTranslation(t.x(), t.y(), t.z());
    this->setOrientationQuat(q.x(), q.y(), q.z(), q.w());
    this->setScale(s.x(), s.y(), s.z());
    
    
    // So... what to do?
    
    /*
    translation_ = t;
    quat_ = q;
    scale_ = s;
    orientation_ = Vec3inDegrees(QuatToEuler(quat_));

    
    // can't do this yet, because a message will result!
    if (orientationMode_!=NORMAL)
    {
       // applyOrientationMode();
    }
    
    BROADCAST(this, "sfff", "setTranslation", translation_.x(), translation_.y(), translation_.z());
    BROADCAST(this, "sffff", "setOrientationQuat", quat_.x(), quat_.y(), quat_.z(), quat_.w());
    BROADCAST(this, "sfff", "setScale", scale_.x(), scale_.y(), scale_.z());
    */
    
}
    
void GroupNode::setManipulator(const char *manipulatorType)
{
    //if (this->owner_.valid()) return;
    if (std::string(manipulatorType)==manipulatorType_) return;
    
    manipulatorType_ = std::string(manipulatorType);
    BROADCAST(this, "ss", "setManipulator", getManipulator());

    // set the flag, which is checked during the updateCallback, and calls the
    // drawManipulator method below
    manipulatorUpdateFlag_ = true;
}

void GroupNode::drawManipulator()
{
    // remove the previously attached dragger (if one exists):
    if (dragger_.valid())
    {
        this->removeChild(dragger_.get());
        dragger_ = 0;
    }

    dragger_ = createDragger(manipulatorType_, this->getBound().radius(), this->getTranslation(), this->getScale());
    
    if (dragger_.valid())
    {
        this->addChild(dragger_.get());
        
        dragger_->addDraggerCallback(new DraggerCallback(this));

        // we want the dragger to handle it's own events automatically
        dragger_->setHandleEvents(true);
        
        // if we don't set an activation key or mod mask then any mouse click on
        // the dragger will activate it, however if do define either of ActivationModKeyMask or
        // and ActivationKeyEvent then you'll have to press either than mod key or the specified key to
        // be able to activate the dragger when you mouse click on it.  Please note the follow allows
        // activation if either the ctrl key or the 'a' key is pressed and held down.
        dragger_->setActivationModKeyMask(osgGA::GUIEventAdapter::MODKEY_CTRL);
        dragger_->setActivationKeyEvent('a');

    }
      
    // note: setAttachmentNode will call updateNodePath
    //this->setAttachmentNode(clipNode_.get());
    //this->updateNodePath();
}

    
// *****************************************************************************

osg::Matrix GroupNode::getGlobalMatrix()
{
    if (!this->reportMode_)
    {
        // might not be up-to-date, so force an update:
        globalMatrix_ = osg::computeLocalToWorld(this->currentNodePath_);
    }

    return globalMatrix_;
}

osg::Vec3 GroupNode::getCenter() const
{
    const osg::BoundingSphere& bs = this->getBound();
    //osg::BoundingSphere& bs = this->computeBound();
    return bs.center();
}


bool GroupNode::dumpGlobals(bool forced)
{
    // The "forced" parameter means that we do the dump even if there has been
    // no change. The "forced" flag should NEVER be used in the updateCallback,
    // which occurs very frequently. The stateDump() method will however force
    // an update of the current global parameters

    //if (this->_reportGlobals)
    if (this->reportMode_)
    {

        // position & rotation: (should we get position from centre of boundingsphere?
        osg::Matrix myMatrix = osg::computeLocalToWorld(this->currentNodePath_);
        if ((myMatrix != this->globalMatrix_) || forced)
        {
            this->globalMatrix_ = myMatrix;
            
            //TODO: use decompose!
            //osg::Vec3 myPos = myMatrix.getTrans();
            //osg::Vec3 myRot = Vec3inDegrees(QuatToEuler(myMatrix.getRotate()));

            osg::Vec3 myPos;
            osg::Quat myQuat;
            osg::Vec3 myScale;
            osg::Quat myScaleOrientation;
            this->globalMatrix_.decompose(myPos, myQuat, myScale, myScaleOrientation);
            osg::Vec3 myRot = Vec3inDegrees(QuatToEuler(myQuat));
            

            BROADCAST(this, "sffffff", "global6DOF", myPos.x(), myPos.y(), myPos.z(), myRot.x(), myRot.y(), myRot.z());

            if (this->reportMode_ == GLOBAL_ALL)
            {
                osg::ComputeBoundsVisitor *vst = new osg::ComputeBoundsVisitor;
                this->accept(*vst);
                osg::BoundingBox bb = vst->getBoundingBox();
                osg::Vec3 myScale = osg::Vec3(bb.xMax()-bb.xMin(), bb.yMax()-bb.yMin(), bb.zMax()-bb.zMin());
                if ((myScale != this->_globalScale) || forced)
                {
                    this->_globalScale = myScale;
                    BROADCAST(this, "sfff", "globalScale", _globalScale.x(), _globalScale.y(), _globalScale.z());
                }
            }
        }


        if (this->reportMode_ == GLOBAL_ALL)
        {
            const osg::BoundingSphere& bs = this->getBound();
            if ((bs.radius() != globalRadius_) || forced)
            {
                globalRadius_ = bs.radius();
                BROADCAST(this, "sf", "globalRadius", globalRadius_);
            }
        }

        return true;

    }
    return false;
}

void GroupNode::stateDump ()
{
    ReferencedNode::stateDump();
    dumpGlobals(true);
}

// *****************************************************************************


std::vector<lo_message> GroupNode::getState () const
{
    // inherit state from base class
    std::vector<lo_message> ret = ReferencedNode::getState();

    lo_message msg;
    osg::Vec3 v;

	msg = lo_message_new();
	lo_message_add(msg,  "ss", "setStateSet", getStateSetSymbol()->s_name);
	ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "si", "setComputationMode", getComputationMode());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "sf", "setUpdateRate", getUpdateRate());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "si", "setReportMode", getReportMode());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "si", "setInteractionMode", getInteractionMode());
    ret.push_back(msg);

    msg = lo_message_new();
    v = getOrientation();
    lo_message_add(msg, "si", "setOrientationMode", getOrientationMode());
    ret.push_back(msg);
    
    msg = lo_message_new();
    v = getOrientation();
    lo_message_add(msg, "ss", "setOrientationTarget", getOrientationTarget());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "sfff", "setClipping", clipping_.x(), clipping_.y(), clipping_.z());
    ret.push_back(msg);

    msg = lo_message_new();
    v = getTranslation();
    lo_message_add(msg, "sfff", "setTranslation", v.x(), v.y(), v.z());
    ret.push_back(msg);

    msg = lo_message_new();
    v = getOrientation();
    lo_message_add(msg, "sfff", "setOrientation", v.x(), v.y(), v.z());
    ret.push_back(msg);

    msg = lo_message_new();
    v = getScale();
    lo_message_add(msg, "sfff", "setScale", v.x(), v.y(), v.z());
    ret.push_back(msg);

    msg = lo_message_new();
    v = getVelocity();
    lo_message_add(msg, "sfff", "setVelocity", v.x(), v.y(), v.z());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "si", "setVelocityMode", getVelocityMode());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "sf", "setDamping", getDamping());
    ret.push_back(msg);


    //lo_message_free(msg);

    return ret;
}

} // end of namespace spin


