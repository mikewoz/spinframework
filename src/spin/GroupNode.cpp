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

#include "GroupNode.h"
#include "SceneManager.h"
#include "osgUtil.h"



using namespace std;




// ***********************************************************
// constructor:
GroupNode::GroupNode (SceneManager *sceneManager, char *initID) : ReferencedNode(sceneManager, initID)
{
	nodeType = "GroupNode";
	this->setName(string(id->s_name) + ".GroupNode");

	_reportGlobals = false;
	_reportMode = GroupNode::GLOBAL_6DOF;

	
	mainTransform = new osg::PositionAttitudeTransform();
	mainTransform->setName(string(id->s_name) + ".mainTransform");
	this->addChild(mainTransform.get());


	// When children are attached to this, they get added to the attachNode:
	// NOTE: by changing this, we MUST override the updateNodePath() method!
	setAttachmentNode(mainTransform.get());

	// keep a timer for velocity calculation:
	lastTick = osg::Timer::instance()->tick();

}

// ***********************************************************
// destructor
GroupNode::~GroupNode()
{

}


void GroupNode::callbackUpdate()
{

	dumpGlobals(false); // never force globals here


	// Now we need to update translation based on velocity. Since it's in m/s,
	// we need to find out how many seconds passed since the last time this was
	// called, and move by _velocity*dt
    if ( !sceneManager->isSlave() && (_velocity!=osg::Vec3(0,0,0)) )
	{
	    osg::Timer_t tick = osg::Timer::instance()->tick();
		float dt = osg::Timer::instance()->delta_s(lastTick,tick);
		if (dt > 0.05) // only update when dt is at least 0.05s (ie 20hz):
		{
            this->move( _velocity.x()*dt, _velocity.y()*dt, _velocity.z()*dt );
            lastTick = tick;
		}
	}

}


void GroupNode::updateNodePath()
{
	currentNodePath.clear();
	if ((parent!=WORLD_SYMBOL) && (parent!=NULL_SYMBOL))
	{
		osg::ref_ptr<ReferencedNode> parentNode = parent->s_thing;
		if (parentNode.valid())
		{
			currentNodePath = parentNode->currentNodePath;
		}
	}

	// here, the nodePath includes the base osg::group, PLUS the mainTransform
	currentNodePath.push_back(this);
	currentNodePath.push_back(mainTransform.get());

	// now update NodePaths for all children:
	updateChildNodePaths();

	/*
	osg::NodePath::iterator iter;
	std::cout << "nodepath for " << id->s_name << ":" << std::endl;
	for (iter = currentNodePath.begin(); iter!=currentNodePath.end(); iter++)
	{
		std::cout << " > " << (*iter)->getName() << std::endl;
	}
	*/
}


// ***********************************************************
// ***********************************************************

void GroupNode::reportGlobals (int b)
{
	if (this->_reportGlobals != (bool)b)
	{
		this->_reportGlobals = (bool) b;
		BROADCAST(this, "si", "reportGlobals", (int) this->_reportGlobals);
	}
}

void GroupNode::setReportMode (globalsReportMode mode)
{
	if (this->_reportMode != (bool)mode)
	{
		this->_reportMode = mode;
		BROADCAST(this, "si", "setReportMode", (int) this->_reportMode);
	}
}



void GroupNode::setTranslation (float x, float y, float z)
{
	mainTransform->setPosition(osg::Vec3d(x,y,z));

	BROADCAST(this, "sfff", "setTranslation", x, y, z);
}


void GroupNode::setOrientation (float p, float r, float y)
{

	_orientation = osg::Vec3(p, r, y);

	osg::Quat q = osg::Quat( osg::DegreesToRadians(p), osg::Vec3d(1,0,0),
							 osg::DegreesToRadians(r), osg::Vec3d(0,1,0),
							 osg::DegreesToRadians(y), osg::Vec3d(0,0,1));

	mainTransform->setAttitude(q);

	BROADCAST(this, "sfff", "setOrientation", p, r, y);
}
/*
void GroupNode::setOrientationQuat (float x, float y, float z, float w)
{

	_orientation = osg::Vec3(p, r, y);

	osg::Quat q = osg::Quat( osg::DegreesToRadians(p), osg::Vec3d(1,0,0),
							 osg::DegreesToRadians(r), osg::Vec3d(0,1,0),
							 osg::DegreesToRadians(y), osg::Vec3d(0,0,1));

	mainTransform->setAttitude(osg::Quat(x,y,z,w));

	BROADCAST(this, "sfff", "setOrientation", p, r, y);
}
*/

void GroupNode::setVelocity (float dx, float dy, float dz)
{
	osg::Vec3 newVelocity = osg::Vec3(dx,dy,dz);
	
	if (newVelocity != _velocity)
	{
		_velocity = newVelocity;
		BROADCAST(this, "sfff", "setVelocity", dx, dy, dz);
	}
}

void GroupNode::move (float x, float y, float z)
{
	osg::Vec3 newPos = mainTransform->getPosition() + ( mainTransform->getAttitude() * osg::Vec3(x,y,z) );
	setTranslation(newPos.x(), newPos.y(), newPos.z());
}

void GroupNode::rotate (float p, float r, float y)
{
	this->setOrientation(_orientation.x()+p, _orientation.y()+r, _orientation.z()+y);
}

// *****************************************************************************




std::vector<lo_message> GroupNode::getState ()
{
	// inherit state from base class
	std::vector<lo_message> ret = ReferencedNode::getState();

	lo_message msg;
	osg::Vec3 v;


	msg = lo_message_new();
	lo_message_add(msg, "si", "reportGlobals",(int) this->_reportGlobals);
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "si", "reportMode",(int) this->_reportMode);
	ret.push_back(msg);
	
	msg = lo_message_new();
	v = this->getTranslation();
	lo_message_add(msg, "sfff", "setTranslation", v.x(), v.y(), v.z());
	ret.push_back(msg);

	msg = lo_message_new();
	v = this->getOrientation();
	lo_message_add(msg, "sfff", "setOrientation", v.x(), v.y(), v.z());
	ret.push_back(msg);

	msg = lo_message_new();
	v = this->getVelocity();
	lo_message_add(msg, "sfff", "setVelocity", v.x(), v.y(), v.z());
	ret.push_back(msg);


	//lo_message_free(msg);

	return ret;
}

// *****************************************************************************
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
