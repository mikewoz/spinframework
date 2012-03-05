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

//#include <osg/Matrix>

#include "osgUtil.h"
#include "AnimationNode.h"
#include "spinApp.h"
#include "spinBaseContext.h"
#include "MediaManager.h"

namespace spin
{

class SceneManager;

// *****************************************************************************
// constructor:
AnimationNode::AnimationNode (SceneManager *sceneManager, char *initID) : GroupNode(sceneManager, initID)
{
    this->setName(std::string(id->s_name) + ".AnimationNode");
    nodeType = "AnimationNode";

    _updateRate = 15; // hz
    _play = false;
    _record = false;

    _animationPath = new osg::AnimationPath;
    _animationPath->setLoopMode(osg::AnimationPath::LOOP);

    _lastTick = osg::Timer::instance()->tick();

    /*
    _animationPathCallback = new osg::AnimationPathCallback(_animationPath.get(),0.0,1.0);

    mainTransform->setDataVariance(osg::Object::DYNAMIC);
    //mainTransform->setUserData( dynamic_cast<osg::Referenced*>(mainTransform.get()) );
    mainTransform->setUpdateCallback(_animationPathCallback.get());

    _animationPathCallback->setPause(true);
     */
}

// *****************************************************************************
// destructor
AnimationNode::~AnimationNode()
{

}



// *****************************************************************************

void AnimationNode::callbackUpdate()
{

    ReferencedNode::callbackUpdate();

    if ( spinApp::Instance().getContext()->isServer() and getPlay() 
            and not _animationPath->empty())
    {
        osg::Timer_t tick = osg::Timer::instance()->tick();
        float dt = osg::Timer::instance()->delta_s(_lastTick,tick);

        //if (dt > 0.05) // only update when dt is at least 0.05s (ie 20hz):
        if (dt > (1/_updateRate))
        {
            doUpdate(osg::Timer::instance()->delta_s(_startTime,tick));
            _lastTick = tick;
        }
    }

}

bool AnimationNode::doUpdate(double timestamp)
{
    osg::Matrix myMatrix;
    if (_animationPath->getMatrix(timestamp, myMatrix))
    {
        osg::Vec3 myPos = myMatrix.getTrans();
        osg::Vec3 myRot = Vec3inDegrees(QuatToEuler(myMatrix.getRotate()));
        osg::Vec3 myScl = myMatrix.getScale();

        setTranslation(myPos.x(), myPos.y(), myPos.z());
        setOrientation(myRot.x(), myRot.y(), myRot.z());
        setScale(myScl.x(), myScl.y(), myScl.z());

        return true;
    }


    /*
    AnimationPath::ControlPoint cp;
    if (_animationPath->getInterpolatedControlPoint(currentTime,cp))
    {
        mainTransform.setPosition(cp.getPosition());
        mainTransform.setAttitude(cp.getRotation());
        mainTransform.setScale(cp.getScale());
    }
    */

    return false;
}

// *****************************************************************************


void AnimationNode::setIndex (float index)
{
    //setPlay(0);
    setRecord(0);

    // make sure index is in normalized range [0,1]:
    if (index > 1) index=1.0;
    if (index < 0) index=0.0;

    double scale = _animationPath->getLastTime() - _animationPath->getFirstTime();

    doUpdate(index * scale);

    BROADCAST(this, "sf", "setIndex", index);
}


void AnimationNode::setUpdateRate (float hz)
{
    _updateRate = hz;
    BROADCAST(this, "sf", "setUpdateRate", hz);
}

void AnimationNode::setPlay (int p)
{
    if (_play != p)
    {
        _play = (bool)p;

        if (_play)
        {
            setRecord(0);
            _startTime = osg::Timer::instance()->tick();
        }

        BROADCAST(this, "si", "setPlay", getPlay());
    }
}

void AnimationNode::setRecord (int r)
{
    if (_record != r)
    {
        _record = (bool)r;

        if (_record)
        {
            setPlay(0);
            clear();
            _startTime = osg::Timer::instance()->tick();
            storeCurrentPosition(0.0); // record current pos as start position
        }

        BROADCAST(this, "si", "setRecord", getRecord());
    }
}

void AnimationNode::setLoopMode (LoopMode mode)
{
    if (_animationPath->getLoopMode() != (osg::AnimationPath::LoopMode)mode)
    {
        _animationPath->setLoopMode((osg::AnimationPath::LoopMode)mode);
        BROADCAST(this, "si", "setLoopMode", getLoopMode());
    }
}

void AnimationNode::setTranslation (float x, float y, float z)
{
    osg::Vec3 newTranslation = osg::Vec3(x,y,z);

    if (newTranslation != getTranslation())
    {
        // first, call the parent method
        GroupNode::setTranslation(x,y,z);

        // now, if _record mode is set, store the updated position
        if (_record) storeCurrentPosition();
    }
}

void AnimationNode::setOrientation (float p, float r, float y)
{
    osg::Vec3 newOrientation = osg::Vec3(p, r, y);

    if (newOrientation != getOrientation())
    {
        // first, call the parent method
        GroupNode::setOrientation(p,r,y);

        // now, if _record mode is set, store the updated position
        if (_record) storeCurrentPosition();
    }
}

void AnimationNode::setScale (float x, float y, float z)
{
    osg::Vec3 newScale = osg::Vec3(x,y,z);

    if (newScale != getScale())
    {
        // first, call the parent method
        GroupNode::setScale(x,y,z);

        // now, if _record mode is set, store the updated position
        if (_record) storeCurrentPosition();
    }
}

// *****************************************************************************

void AnimationNode::storeCurrentPosition()
{
    double dt = osg::Timer::instance()->delta_s(_startTime,osg::Timer::instance()->tick());
    storeCurrentPosition( dt );
}

void AnimationNode::storeCurrentPosition(double timestamp)
{
    /*
    _animationPath->insert(timestamp, osg::AnimationPath::ControlPoint(
            mainTransform->getPosition(),
            mainTransform->getAttitude(),
            mainTransform->getScale())
    );
    */

    osg::Vec3 myPos = mainTransform->getPosition();
    osg::Quat myRot = mainTransform->getAttitude();
    osg::Vec3 myScl = mainTransform->getScale();
    controlPoint( timestamp, myPos.x(), myPos.y(), myPos.z(), myRot.x(), myRot.y(), myRot.z(), myRot.w(), myScl.x(), myScl.y(), myScl.z() );
}

void AnimationNode::controlPoint (double timestamp, float x, float y, float z, float rotX, float rotY, float rotZ, float rotW, float scaleX, float scaleY, float scaleZ)
{
    _animationPath->insert(timestamp, osg::AnimationPath::ControlPoint(
            osg::Vec3(x,y,z),
            osg::Quat(rotX,rotY,rotZ,rotW),
            osg::Vec3(scaleX,scaleY,scaleZ))
    );

    BROADCAST(this, "sdffffffffff", "controlPoint", timestamp, x, y, z, rotX, rotY, rotZ, rotW, scaleX, scaleY, scaleZ);

}

// *****************************************************************************
std::vector<lo_message> AnimationNode::getState () const
{
    // inherit state from base class
    std::vector<lo_message> ret = GroupNode::getState();

    lo_message msg;

    msg = lo_message_new();
    lo_message_add(msg, "si", "setPlay", getPlay());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "sf", "setUpdateRate", getUpdateRate());
    ret.push_back(msg);

    /*
    msg = lo_message_new();
    lo_message_add(msg, "si", "setRecord", getRecord());
    ret.push_back(msg);
    */

    msg = lo_message_new();
    lo_message_add(msg, "si", "setLoopMode", getLoopMode());
    ret.push_back(msg);

    osg::AnimationPath::TimeControlPointMap::iterator itr;
    osg::AnimationPath::TimeControlPointMap controlPoints = _animationPath->getTimeControlPointMap();
    for (itr=controlPoints.begin(); itr!=controlPoints.end(); itr++)
    {
        msg = lo_message_new();;
        osg::Vec3 pos = (*itr).second.getPosition();
        osg::Quat rot = (*itr).second.getRotation();
        osg::Vec3 scl = (*itr).second.getScale();
        lo_message_add(msg, "sdffffffffff", "controlPoint", (*itr).first, pos.x(), pos.y(), pos.z(), rot.x(), rot.y(), rot.z(), rot.w(), scl.x(), scl.y(), scl.z());
        ret.push_back(msg);
    }

    return ret;
}

} // end of namespace spin

