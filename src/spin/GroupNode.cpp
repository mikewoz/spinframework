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

#include <osgManipulator/TabBoxDragger>
#include <osgManipulator/TabBoxTrackballDragger>
#include <osgManipulator/TabPlaneDragger>
#include <osgManipulator/TabPlaneTrackballDragger>
#include <osgManipulator/TrackballDragger>
#include <osgManipulator/Translate1DDragger>
#include <osgManipulator/Translate2DDragger>
#include <osgManipulator/TranslateAxisDragger>

#include "GroupNode.h"
#include "SceneManager.h"
#include "spinApp.h"
#include "spinBaseContext.h"
#include "osgUtil.h"
#include "UserNode.h"
#include "spinApp.h"

using namespace std;

namespace spin
{

// ***********************************************************
// constructor:
GroupNode::GroupNode (SceneManager *sceneManager, char *initID) : ReferencedNode(sceneManager, initID)
{
    nodeType = "GroupNode";
    this->setName(string(id->s_name) + ".GroupNode");

    //_reportGlobals = false;
    _reportMode = GroupNode::NONE;
    _interactionMode = GroupNode::STATIC;


    mainTransform = new osg::PositionAttitudeTransform();
    mainTransform->setName(string(id->s_name) + ".mainTransform");
    //mainTransform->setAttitude(osg::Quat(0.0,0.0,0.0,0.0));
    //mainTransform->setPosition(osg::Vec3(0.0,0.0,0.0));
    this->addChild(mainTransform.get());

    manipulatorTransform = new osg::MatrixTransform();
    manipulatorTransform->setName(string(id->s_name) + ".manipulatorTransform");
    mainTransform->addChild(manipulatorTransform.get());
    

    clipNode = new osg::ClipNode();
    clipNode->setCullingActive(false);
    clipNode->setName(string(id->s_name) + ".clipNode");
    manipulatorTransform->addChild(clipNode.get());

    _velocity = osg::Vec3(0.0,0.0,0.0);
    _velocityMode = GroupNode::TRANSLATE;
    _spin = osg::Vec3(0.0,0.0,0.0);
    _damping = 0.0;

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
	ReferencedNode::updateNodePath(false);
	/*
    currentNodePath.clear();
    if ((parent!=WORLD_SYMBOL) && (parent!=NULL_SYMBOL))
    {
        osg::ref_ptr<ReferencedNode> parentNode = dynamic_cast<ReferencedNode*>(parent->s_thing);
        if (parentNode.valid())
        {
            currentNodePath = parentNode->currentNodePath;
        }
    }

    // here, the nodePath includes the base osg::group, PLUS the mainTransform
    // and clipNode
    currentNodePath.push_back(this);
    */
    currentNodePath.push_back(mainTransform.get());
    currentNodePath.push_back(clipNode.get());

    // now update NodePaths for all children:
    if (updateChildren) updateChildNodePaths();

    /*
    osg::NodePath::iterator iter;
    std::cout << "nodepath for " << id->s_name << ":" << std::endl;
    for (iter = currentNodePath.begin(); iter!=currentNodePath.end(); iter++)
    {
        std::cout << " > " << (*iter)->getName() << std::endl;
    }
    */

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

                        vector<osg::Vec4>::iterator it;
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
                    osg::Vec3 newPos = mainTransform->getPosition() + motionVec;
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

    if (owner.valid()) std::cout << "   owner: " << owner->id->s_name << std::endl;
    else std::cout << "   owner: NULL" << std::endl;

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

void GroupNode::setInteractionMode (interactionMode mode)
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


void GroupNode::setTranslation (float x, float y, float z)
{
    osg::Vec3 newTranslation = osg::Vec3(x,y,z);

    if (newTranslation != getTranslation())
    {
        mainTransform->setPosition(newTranslation);
        BROADCAST(this, "sfff", "setTranslation", x, y, z);
    }
}

void GroupNode::setOrientationQuat (float x, float y, float z, float w)
{
	osg::Quat newQuat = osg::Quat(x,y,z,w);

    if (newQuat != mainTransform->getAttitude())
    {
        mainTransform->setAttitude(newQuat);
        _orientation = Vec3inDegrees(QuatToEuler(newQuat));

        BROADCAST(this, "sffff", "setOrientationQuat", x, y, z, w);
    }

}

void GroupNode::setOrientation (float p, float r, float y)
{
    osg::Vec3 newOrientation = osg::Vec3(p, r, y);

    if (newOrientation != getOrientation())
    {
        _orientation = newOrientation;
        osg::Quat q = osg::Quat( osg::DegreesToRadians(p), osg::Vec3d(1,0,0),
                                 osg::DegreesToRadians(r), osg::Vec3d(0,1,0),
                                 osg::DegreesToRadians(y), osg::Vec3d(0,0,1));
        mainTransform->setAttitude(q);
        BROADCAST(this, "sfff", "setOrientation", p, r, y);
    }
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

    if (newScale != getScale())
    {
        mainTransform->setScale(newScale);
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
    osg::Vec3 newPos = mainTransform->getPosition() + osg::Vec3(x,y,z);
    setTranslation(newPos.x(), newPos.y(), newPos.z());
}


void GroupNode::move (float x, float y, float z)
{
    // take the orientation into account, and move along that vector:
    osg::Vec3 newPos = mainTransform->getPosition() + ( mainTransform->getAttitude() * osg::Vec3(x,y,z) );
    setTranslation(newPos.x(), newPos.y(), newPos.z());
}


void GroupNode::rotate (float p, float r, float y)
{
    this->setOrientation(_orientation.x()+p, _orientation.y()+r, _orientation.z()+y);
}

// *****************************************************************************

    
osgManipulator::Dragger* createDragger(const std::string& name)
{
    osgManipulator::Dragger* dragger = 0;
    if ("TabPlaneDragger" == name)
    {
        osgManipulator::TabPlaneDragger* d = new osgManipulator::TabPlaneDragger();
        d->setupDefaultGeometry();
        dragger = d;
    }
    else if ("TabPlaneTrackballDragger" == name)
    {
        osgManipulator::TabPlaneTrackballDragger* d = new osgManipulator::TabPlaneTrackballDragger();
        d->setupDefaultGeometry();
        dragger = d;
    }
    else if ("TabBoxTrackballDragger" == name)
    {
        osgManipulator::TabBoxTrackballDragger* d = new osgManipulator::TabBoxTrackballDragger();
        d->setupDefaultGeometry();
        dragger = d;
    }
    else if ("TrackballDragger" == name)
    {
        osgManipulator::TrackballDragger* d = new osgManipulator::TrackballDragger();
        d->setupDefaultGeometry();
        dragger = d;
    }
    else if ("Translate1DDragger" == name)
    {
        osgManipulator::Translate1DDragger* d = new osgManipulator::Translate1DDragger();
        d->setupDefaultGeometry();
        dragger = d;
    }
    else if ("Translate2DDragger" == name)
    {
        osgManipulator::Translate2DDragger* d = new osgManipulator::Translate2DDragger();
        d->setupDefaultGeometry();
        dragger = d;
    }
    else if ("TranslateAxisDragger" == name)
    {
        osgManipulator::TranslateAxisDragger* d = new osgManipulator::TranslateAxisDragger();
        d->setupDefaultGeometry();
        dragger = d;
    }
    else if ("TabBoxDragger" == name)
    {
        osgManipulator::TabBoxDragger* d = new osgManipulator::TabBoxDragger();
        d->setupDefaultGeometry();
        dragger = d;
    }
    
    if (dragger)
    {
        // turn off lighing on dragger
        osg::StateSet *ss = dragger->getOrCreateStateSet();
        ss->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
    }
    
    return dragger;
}

    
void GroupNode::setManipulator(const char *manipulatorType)
{
    
    if (this->owner.valid()) return;
    
    _manipulatorType = std::string(manipulatorType);
    BROADCAST(this, "ss", "setManipulator", getManipulator());

    // only do the dragger stuff on the viewer side:
    if ( !spinApp::Instance().getContext()->isServer() )
    {
    
        if (dragger.valid())
        {            
            // Apply the transform of the matrixTransform (updated by the
            // dragger) to the mainTransform, but, we need to do this with
            // setTranslation, setOrientation, and setScale calls so that the 
            // server finally gets notified:
            
            osg::Matrix m = manipulatorTransform->getMatrix();
            //if (!osg::Matrix::invert(m)) std::cout << "couldn't invert matrix" <<std::endl;
            osg::Vec3 t = mainTransform->getPosition() + m.getTrans();
            osg::Quat q = mainTransform->getAttitude() * m.getRotate();
            osg::Vec3 s = osg::Vec3(mainTransform->getScale().x() * m.getScale().x(),
                                    mainTransform->getScale().y() * m.getScale().y(),
                                    mainTransform->getScale().z() * m.getScale().z());
            
            /*
            osg::PositionAttitudeTransform *pat = manipulatorTransform->asPositionAttitudeTransform();
            if (!pat)
                std::cout << "error: can't get PAT from manipulatorTransform" << std::endl;
            
            osg::Vec3 t = mainTransform->getPosition() + pat->getPosition();
            osg::Quat q = mainTransform->getAttitude() * pat->getAttitude();
            osg::Vec3 s = osg::Vec3(mainTransform->getScale().x() * pat->getScale().x(),
                                    mainTransform->getScale().y() * pat->getScale().y(),
                                    mainTransform->getScale().z() * pat->getScale().z());
        */
            
            // unlock the node:
            spinApp::Instance().NodeMessage(this->getID().c_str(), "ssi", "setLock", spinApp::Instance().getUserID().c_str(), 0, SPIN_ARGS_END);
            
            spinApp::Instance().NodeMessage(this->getID().c_str(), "sfff", "setTranslation", t.x(), t.y(), t.z(), SPIN_ARGS_END);
            spinApp::Instance().NodeMessage(this->getID().c_str(), "sffff", "setOrientationQuat", q.x(), q.y(), q.z(), q.w(), SPIN_ARGS_END);
            spinApp::Instance().NodeMessage(this->getID().c_str(), "sfff", "setScale", s.x(), s.y(), s.z(), SPIN_ARGS_END);

            
            // return the manipulatorTransform to an identity matrix, and remove
            // the dragger:
            manipulatorTransform->setMatrix(osg::Matrix::identity());
            mainTransform->removeChild(dragger.get());
            dragger = 0;

        }
        
        dragger = createDragger(manipulatorType);
        if (dragger.valid())
        {
            // tell server that this viewer's UserNode is now the owner:
            spinApp::Instance().NodeMessage(this->getID().c_str(), "ssi", "setLock", spinApp::Instance().getUserID().c_str(), 1, SPIN_ARGS_END);
            
            mainTransform->addChild(dragger.get());
            
            float scale = this->getBound().radius() * 1.6;
            dragger->setMatrix(osg::Matrix::scale(scale, scale, scale) *
                               osg::Matrix::translate(this->getBound().center()));
            
            dragger->addTransformUpdating(manipulatorTransform.get());
            
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
    }
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
            osg::Vec3 myPos = myMatrix.getTrans();
            osg::Vec3 myRot = Vec3inDegrees(QuatToEuler(myMatrix.getRotate()));

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


