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
GroupNode::GroupNode (SceneManager *sceneManager, char *initID) : ReferencedNode(sceneManager, initID)
{
    nodeType = "GroupNode";
    this->setName(getID() + ".GroupNode");

    //_reportGlobals = false;
    _reportMode = GroupNode::NONE;
    _interactionMode = GroupNode::STATIC;
    orientationMode_ = GroupNode::NORMAL;
    
    orientationTarget_ = gensym("NULL");
    manipulatorType_ = "NULL";
    manipulatorUpdateFlag_ = false;
    manipulatorShadowCopy_ = true;

    mainTransform = new osg::MatrixTransform();
    mainTransform->setName(getID() + ".mainTransform");
    //mainTransform->setAttitude(osg::Quat(0.0,0.0,0.0,0.0));
    //mainTransform->setPosition(osg::Vec3(0.0,0.0,0.0));
    this->addChild(mainTransform.get());

    

    clipNode = new osg::ClipNode();
    clipNode->setCullingActive(false);
    clipNode->setName(getID() + ".clipNode");
    mainTransform->addChild(clipNode.get());
    
    scale_ = osg::Vec3(1.0,1.0,1.0);
    _velocity = osg::Vec3(0.0,0.0,0.0);
    _velocityMode = GroupNode::TRANSLATE;
    _spin = osg::Vec3(0.0,0.0,0.0);
    _damping = 0.0;
    
    updateMatrix();

    // When children are attached to this, they get added to the attachNode:
    // NOTE: by changing this, we MUST override the updateNodePath() method!
    setAttachmentNode(clipNode.get());

    // keep a timer for velocity calculation:
    lastTick = osg::Timer::instance()->tick();
}

// ***********************************************************
// destructor
GroupNode::~GroupNode()
{

}

#define EPSILON 0.0001

void GroupNode::callbackUpdate()
{
    ReferencedNode::callbackUpdate();
    
    if (manipulatorUpdateFlag_)
    {
        drawManipulator();
        manipulatorUpdateFlag_ = false;
    }

    //printf("GroupNode: damping = %f\n", _damping);

    dumpGlobals(false); // never force globals here

    // Now we need to update translation/orientation based on our velocity/spin.
    // We find out how many seconds passed since the last time this was called,
    // and move by _velocity*dt (ie, m/s) and rotate by _spin*dt (ie, deg/sec)
    if ( spinApp::Instance().getContext()->isServer() )
    {
        osg::Timer_t tick = osg::Timer::instance()->tick();
        float dt = osg::Timer::instance()->delta_s(lastTick,tick);
        if (dt > 0.05) // only update when dt is at least 0.05s (ie 20hz):
        //if (dt > 0.1) // only update when dt is at least 0.1s (ie 10hz):
        {

            if (_velocity.length() > EPSILON) // != osg::Vec3(0,0,0))
            {
            	if (this->_velocityMode==GroupNode::TRANSLATE)
            		this->translate( _velocity.x()*dt, _velocity.y()*dt, _velocity.z()*dt );
            	else
            		this->move( _velocity.x()*dt, _velocity.y()*dt, _velocity.z()*dt );

                if (_damping > EPSILON)
                {
                    double dv = 1 - (_damping*dt);
                    this->setVelocity(_velocity.x()*dv, _velocity.y()*dv, _velocity.z()*dv);
                }
            }
            else _velocity = osg::Vec3(0,0,0);

            if (_spin.length() > EPSILON) // != osg::Vec3(0,0,0))
            {
                this->rotate( _spin.x()*dt, _spin.y()*dt, _spin.z()*dt );
                if (_damping > EPSILON)
                {
                    double ds = 1 - (_damping*dt);
                    this->setSpin(_spin.x()*ds, _spin.y()*ds, _spin.z()*ds);
                }
            }
            else _spin = osg::Vec3(0,0,0);

            lastTick = tick;
        }
    }
}

void GroupNode::updateNodePath(bool updateChildren)
{
    // get node path from parent class, but with the false flag so that children
    // are not updated yet:
	ReferencedNode::updateNodePath(false);

    /*
    if (dragger.valid())
        currentNodePath.push_back(manipulatorTransform.get());
    else
        currentNodePath.push_back(mainTransform.get());
    */
    currentNodePath.push_back(mainTransform.get());
    
    currentNodePath.push_back(clipNode.get());
    
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
    osg::ref_ptr<UserNode> user = dynamic_cast<UserNode*>(sceneManager->getNode(userString));
    if (!user.valid()) return;

    if (_interactionMode==DRAG || _interactionMode==THROW)
    {
        switch(event)
        {
            case(osgGA::GUIEventAdapter::PUSH):

                if (!this->owner.valid())
                {
                    this->setVelocity(0,0,0);
                    this->setSpin(0,0,0);
                    _trajectory.clear();
                }

                break;

            case(osgGA::GUIEventAdapter::RELEASE):

                if (this->owner == user)
                {
                    if (_interactionMode==THROW)
                    {
                        // Take average of stored motion vectors, and set velocity in
                        // that direction. Note: _damping should be set so that node
                        // gradually stops:

                        double currentTime = osg::Timer::instance()->time_m();

                        osg::Vec3 vel = osg::Vec3(0,0,0);
                        int count = 0;

                        std::vector<osg::Vec4>::iterator it;
                        for (it=_trajectory.begin(); it!=_trajectory.end(); ++it)
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

                if (this->owner == user)
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
                    _trajectory.insert(_trajectory.begin(), osg::Vec4(motionVec, osg::Timer::instance()->time_m()));
                    if (_trajectory.size() > 5) {
                        _trajectory.pop_back();
                    }
                }

                break;
        }

        // broadcast event
        // NO! can't do this, because we will duplicate move/rotate/etc actions

    }

    else if (_interactionMode==DRAW)
    {
        if ((event==osgGA::GUIEventAdapter::PUSH) || (event==osgGA::GUIEventAdapter::RELEASE))
        {
            if ((int)eData2==osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) _drawMod = 1;
            else if ((int)eData2==osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON) _drawMod = 2;
            else _drawMod = 0;

            BROADCAST(this, "ssifff", "draw", userString, _drawMod, x, y, z);
        }
        else if (event==osgGA::GUIEventAdapter::DRAG)
        {
            BROADCAST(this, "ssifff", "draw", userString, _drawMod, x, y, z);
        }
    }

    // after all is done, we update ownership (if this was a PUSH or RELEASE)
    if (_interactionMode==SELECT || _interactionMode==DRAG || _interactionMode==THROW)
    {
        switch(event)
        {
            case(osgGA::GUIEventAdapter::PUSH):

                // on push, we check if the current node is owned by anyone,
                // and if not, we give ownnerhip to the event's user:
                if (!this->owner.valid())
                {
                    this->owner = user.get();
                    //std::cout << "setting owner of " << id->s_name << " to " << owner->id->s_name << std::endl;
                    BROADCAST(this, "ss", "setOwner", owner->id->s_name);
                    BROADCAST(this, "ssi", "select", userString, 1);
                }
                break;

            case(osgGA::GUIEventAdapter::RELEASE):

                // on release, the user gives up ownership of the node (assuming
                // that he had ownership in the first place):
                if (this->owner == user)
                {
                    //std::cout << "setting owner of " << id->s_name << " to NULL" << std::endl;
                    this->owner = NULL;
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
    osg::ref_ptr<UserNode> user = dynamic_cast<UserNode*>(sceneManager->getNode(userString));
    if (!user.valid()) return;

    if (lock)
    {
        if (owner.valid())
        {
            this->owner = user.get();
            //std::cout << "setting owner of " << id->s_name << " to " << owner->id->s_name << std::endl;
            BROADCAST(this, "ss", "setOwner", owner->id->s_name);
        }
    }
    else
    {
        if (this->owner == user)
        {
            //std::cout << "setting owner of " << id->s_name << " to NULL" << std::endl;
            this->owner = NULL;
            BROADCAST(this, "ss", "setOwner", "NULL");
        }
    }
}

void GroupNode::debug()
{
    ReferencedNode::debug();

    std::cout << "   Manipulator: " << manipulatorType_ << std::endl;
    if (owner.valid()) std::cout << "   owner: " << owner->id->s_name << std::endl;
    else std::cout << "   owner: NULL" << std::endl;
    
    std::cout << "   mainX pos: " << stringify(this->getTranslation()) << std::endl;
    std::cout << "   mainX rot: " << stringify(this->getOrientationQuat()) << std::endl;
    std::cout << "   mainX scl: " << stringify(this->getScale()) << std::endl;
}


// ***********************************************************
// ***********************************************************

/*
void GroupNode::reportGlobals (int b)
{
    if (this->_reportGlobals != (bool)b)
    {
        this->_reportGlobals = (bool) b;
        BROADCAST(this, "si", "reportGlobals", (int) this->_reportGlobals);
    }
}
*/

void GroupNode::setReportMode (globalsReportMode mode)
{
    if (this->_reportMode != (int)mode)
    {
        this->_reportMode = mode;
        BROADCAST(this, "si", "setReportMode", (int) this->_reportMode);
    }
}

void GroupNode::setInteractionMode (InteractionMode mode)
{
    if (this->_interactionMode != (int)mode)
    {

        // if this node previously had an owner, it will be lost with the change
        // of mode
        if (this->owner.valid())
        {
            this->owner = NULL;
            BROADCAST(this, "ss", "owner", "NULL");
        } else if (_interactionMode == DRAW)
        {
            BROADCAST(this, "ssifff", "draw", "NULL", 0, 0.0f, 0.0f, 0.0f);
        }

        this->_interactionMode = mode;



        // set the node mask so that an intersection visitor will only test
        // for interactive nodes (note: nodemask info in spinUtil.h)
        if ((int)_interactionMode > 0)
            this->setNodeMask(INTERACTIVE_NODE_MASK);
        else
            this->setNodeMask(GEOMETRIC_NODE_MASK);

        BROADCAST(this, "si", "setInteractionMode", (int) this->_interactionMode);
        
        // now we need to travel up the scenegraph and set all parents to have
        // an InteractionMode of PASSTHRU, otherwise an intersection traversal
        // will never find this node:
        GroupNode *parent = dynamic_cast<GroupNode*>(this->getParentNode());
        while (parent)
        {
            if (!parent->getInteractionMode())
            {
                parent->setInteractionMode(PASSTHRU);
                continue; // rest of parents will be recursively called
            }
            else parent = dynamic_cast<GroupNode*>(parent->getParentNode());
        }
        
    }
}

void GroupNode::setClipping(float x, float y, float z)
{
    _clipping = osg::Vec3d(x,y,z);

    // remove existing clipping planes:
    while (clipNode->getNumClipPlanes()) clipNode->removeClipPlane((unsigned int)0);

    // add this one:
    if ( (_clipping.x()>0) && (_clipping.y()>0) && (_clipping.z()>0) )
    {
        osg::BoundingBox bb(-_clipping.x(),-_clipping.y(),-_clipping.z(),_clipping.x(),_clipping.y(),_clipping.z());
        clipNode->createClipBox(bb);
    }

    BROADCAST(this, "sfff", "setClipping", _clipping.x(), _clipping.y(), _clipping.z());
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
    mainTransform->setMatrix(matrix);
 
}

void GroupNode::updateDraggerMatrix()
{
    osg::Matrix matrix;
    matrix.preMultTranslate(translation_);
    matrix.preMultScale(dragger->getMatrix().getScale());
    dragger->setMatrix(matrix);
}

void GroupNode::setTranslation (float x, float y, float z)
{
    osg::Vec3 newTranslation = osg::Vec3(x,y,z);

    if (newTranslation != translation_)
    {
        translation_ = newTranslation;
        updateMatrix();

        //if (dragger.valid()) updateDraggerMatrix();
        
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
            _orientation = Vec3inDegrees(QuatToEuler(newQuat));

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

        if (newOrientation != _orientation)
        {
            _orientation = newOrientation;
            quat_ = osg::Quat( osg::DegreesToRadians(p), osg::Vec3d(1,0,0),
                                     osg::DegreesToRadians(r), osg::Vec3d(0,1,0),
                                     osg::DegreesToRadians(y), osg::Vec3d(0,0,1));
            updateMatrix();
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
    if (!spinApp::Instance().getContext()->isServer()) return;

    osg::Vec3 targetPos;
    osg::Matrix thisMatrix = osg::computeLocalToWorld(this->currentNodePath);
    
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
                osg::Matrixd targetMatrix = osg::computeLocalToWorld(target->currentNodePath);
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

    _orientation = osg::Vec3(osg::RadiansToDegrees(aed.y()), 0.0, osg::RadiansToDegrees(aed.x()));
    quat_ = osg::Quat( osg::DegreesToRadians(_orientation.x()), osg::Vec3d(1,0,0),
                             osg::DegreesToRadians(_orientation.y()), osg::Vec3d(0,1,0),
                             osg::DegreesToRadians(_orientation.z()), osg::Vec3d(0,0,1));
    updateMatrix();
    BROADCAST(this, "sfff", "setOrientation", _orientation.x(), _orientation.y(),  _orientation.z());
}


/*
void GroupNode::setOrientationQuat (float x, float y, float z, float w)
{
    osg::Quat = osg::Quat(x,y,z,w);
    _orientation = QuatToEuler(q);
    mainTransform->setAttitude(q);

    BROADCAST(this, "sfff", "setOrientation", _orientatoin.x(), _orientation.y(), _orientation.z());
}
*/

void GroupNode::setScale (float x, float y, float z)
{
    osg::Vec3 newScale = osg::Vec3(x,y,z);

    if (newScale != scale_)
    {
        scale_ = newScale;
        updateMatrix();
        BROADCAST(this, "sfff", "setScale", x, y, z);
    }
}

void GroupNode::setVelocity (float dx, float dy, float dz)
{
    osg::Vec3 newVelocity = osg::Vec3(dx,dy,dz);

    if (newVelocity != _velocity)
    {
        _velocity = newVelocity;
        BROADCAST(this, "sfff", "setVelocity", dx, dy, dz);
    }
}

void GroupNode::setVelocityMode (velocityMode mode)
{
    if (this->_velocityMode != (int)mode)
    {
        this->_velocityMode = mode;
        BROADCAST(this, "si", "setVelocityMode", (int) this->_velocityMode);
    }
}


void GroupNode::setSpin (float dp, float dr, float dy)
{
    osg::Vec3 newSpin = osg::Vec3(dp,dr,dy);

    if (newSpin != _spin)
    {
        _spin = newSpin;
        BROADCAST(this, "sfff", "setSpin", dp, dr, dy);
    }

}

void GroupNode::setDamping (float d)
{
    if (d != _damping)
    {
        _damping = d;
        BROADCAST(this, "sf", "setDamping", _damping);
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


void GroupNode::rotate (float p, float r, float y)
{
    this->setOrientation(_orientation.x()+p, _orientation.y()+r, _orientation.z()+y);
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

    mainTransform->setMatrix(m);
    
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
    _orientation = Vec3inDegrees(QuatToEuler(quat_));

    
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
    //if (this->owner.valid()) return;
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
    if (dragger.valid())
    {
        this->removeChild(dragger.get());
        dragger = 0;
    }

    dragger = createDragger(manipulatorType_, this->getBound().radius(), this->getTranslation(), this->getScale());
    
    if (dragger.valid())
    {
        this->addChild(dragger.get());
        
        dragger->addDraggerCallback(new DraggerCallback(this));

        // we want the dragger to handle it's own events automatically
        dragger->setHandleEvents(true);
        
        // if we don't set an activation key or mod mask then any mouse click on
        // the dragger will activate it, however if do define either of ActivationModKeyMask or
        // and ActivationKeyEvent then you'll have to press either than mod key or the specified key to
        // be able to activate the dragger when you mouse click on it.  Please note the follow allows
        // activation if either the ctrl key or the 'a' key is pressed and held down.
        dragger->setActivationModKeyMask(osgGA::GUIEventAdapter::MODKEY_CTRL);
        dragger->setActivationKeyEvent('a');

    }
      
    // note: setAttachmentNode will call updateNodePath
    //this->setAttachmentNode(clipNode.get());
    //this->updateNodePath();
}

    
// *****************************************************************************

osg::Matrix GroupNode::getGlobalMatrix()
{
    if (!this->_reportMode)
    {
        // might not be up-to-date, so force an update:
        _globalMatrix = osg::computeLocalToWorld(this->currentNodePath);
    }

    return _globalMatrix;
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
    if (this->_reportMode)
    {

        // position & rotation: (should we get position from centre of boundingsphere?
        osg::Matrix myMatrix = osg::computeLocalToWorld(this->currentNodePath);
        if ((myMatrix != this->_globalMatrix) || forced)
        {
            this->_globalMatrix = myMatrix;
            
            //TODO: use decompose!
            //osg::Vec3 myPos = myMatrix.getTrans();
            //osg::Vec3 myRot = Vec3inDegrees(QuatToEuler(myMatrix.getRotate()));

            osg::Vec3 myPos;
            osg::Quat myQuat;
            osg::Vec3 myScale;
            osg::Quat myScaleOrientation;
            this->_globalMatrix.decompose(myPos, myQuat, myScale, myScaleOrientation);
            osg::Vec3 myRot = Vec3inDegrees(QuatToEuler(myQuat));
            

            BROADCAST(this, "sffffff", "global6DOF", myPos.x(), myPos.y(), myPos.z(), myRot.x(), myRot.y(), myRot.z());

            if (this->_reportMode == GLOBAL_ALL)
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


        if (this->_reportMode == GLOBAL_ALL)
        {
            const osg::BoundingSphere& bs = this->getBound();
            if ((bs.radius() != _globalRadius) || forced)
            {
                _globalRadius = bs.radius();
                BROADCAST(this, "sf", "globalRadius", _globalRadius);
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
    lo_message_add(msg, "sfff", "setClipping", _clipping.x(), _clipping.y(), _clipping.z());
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


