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

#include <osgDB/ReadFile>
#include <osg/Geometry>
#include <osg/Billboard>

#include "osgUtil.h"
#include "AnimationNode.h"
#include "SceneManager.h"
#include "MediaManager.h"



using namespace std;


extern pthread_mutex_t pthreadLock;


// *****************************************************************************
// constructor:
AnimationNode::AnimationNode (SceneManager *sceneManager, char *initID) : GroupNode(sceneManager, initID)
{
	this->setName(string(id->s_name) + ".AnimationNode");
	nodeType = "AnimationNode";

	setReportMode(GroupNode::GLOBAL_6DOF);
	
	_record = false;

	_animationPath = new osg::AnimationPath;
	_animationPath->setLoopMode(osg::AnimationPath::LOOP);
	
	
	_animationPathCallback = new osg::AnimationPathCallback(_animationPath.get(),0.0,1.0);
    
	mainTransform->setDataVariance(osg::Object::DYNAMIC);
	//mainTransform->setUserData( dynamic_cast<osg::Referenced*>(mainTransform.get()) );
	mainTransform->setUpdateCallback(_animationPathCallback.get());

    _animationPathCallback->setPause(true);
}

// *****************************************************************************
// destructor
AnimationNode::~AnimationNode()
{

}

	
// *****************************************************************************

void AnimationNode::setPlay (int i)
{
	if (i)
	{
		_animationPathCallback->setPause(false);
		setRecord(0);
	}
	else {
		_animationPathCallback->setPause(true);
	}
	
	BROADCAST(this, "si", "setPlay", getPlay());
}

void AnimationNode::setRecord (int i)
{
	if (_record!=i)
	{
		_record = (bool)i;
		if (_record)
		{
			setPlay(false);
			_recordStart = osg::Timer::instance()->tick();
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
	// first, call the parent method
	GroupNode::setTranslation(x,y,z);
	
	// now, if _record mode is set, store the updated position
	if (_record) storeCurrentPosition();
}

void AnimationNode::setOrientation (float p, float r, float y)
{
	// first, call the parent method
	GroupNode::setOrientation(p,r,y);
	
	// now, if _record mode is set, store the updated position
	if (_record) storeCurrentPosition();
}

void AnimationNode::setScale (float x, float y, float z)
{
	// first, call the parent method
	GroupNode::setScale(x,y,z);
	
	// now, if _record mode is set, store the updated position
	if (_record) storeCurrentPosition();
}

// *****************************************************************************

void AnimationNode::storeCurrentPosition()
{
	double dt = osg::Timer::instance()->delta_s(_recordStart,osg::Timer::instance()->tick());
	storeCurrentPosition( dt );
}

void AnimationNode::storeCurrentPosition(double timestamp)
{
	_animationPath->insert(timestamp, osg::AnimationPath::ControlPoint(
			mainTransform->getPosition(),
			mainTransform->getAttitude(),
			mainTransform->getScale())
	);
}

void AnimationNode::controlPoint (double timestamp, float x, float y, float z, float rotX, float rotY, float rotZ, float rotW, float scaleX, float scaleY, float scaleZ)
{
	_animationPath->insert(timestamp, osg::AnimationPath::ControlPoint(
			osg::Vec3(x,y,z),
			osg::Quat(rotX,rotY,rotZ,rotW),
			osg::Vec3(scaleX,scaleY,scaleZ))
	);
}

// *****************************************************************************
std::vector<lo_message> AnimationNode::getState ()
{
	// inherit state from base class
	std::vector<lo_message> ret = GroupNode::getState();

	lo_message msg;
	
	
	msg = lo_message_new();
	lo_message_add(msg, "sd", "currentTime", _animationPathCallback->getAnimationTime());
	ret.push_back(msg);
	
	
	msg = lo_message_new();
	lo_message_add(msg, "si", "setPlay", getPlay());
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
