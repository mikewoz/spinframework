// ===================================================================
// Audioscape library for PureData
// Copyright (c) 2007
//
// Collaborators:
//    Shared Reality Lab (SRE), McGill University Centre for Intelligent Machines (CIM)
//       www.cim.mcgill.ca/sre
//    La Société des Arts Technologiques (SAT)
//       www.sat.qc.ca
//
// Project Directors:
//    Science - Jeremy R. Cooperstock (SRE/CIM)
//    Arts - Zack Settel
//
// Conception:
//    Zack Settel
//
// Development Team:
//    Mike Wozniewski (SRE/CIM): Researcher, Head Developer
//    Zack Settel: Artist, Researcher, Audio/DSP programming
//    Jean-Michel Dumas (SAT): Assistant Researcher
//    Mitchel Benovoy (SRE/CIM): Video Texture Programming
//    Stéphane Pelletier (SRE/CIM): Video Texture Programming
//    Pierre-Olivier Charlebois (SRE/CIM): Former Developer
//
// Funding by / Souventionné par:
//    Natural Sciences and Engineering Research Council of Canada (NSERC)
//    Canada Council for the Arts
//    NSERC/Canada Council for the Arts - New Media Initiative
//
// ===================================================================
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
// ===================================================================

 /**
  * The asBasicNode provides basic manipulation operators for translation and
  * rotation as well as storing the mapping plugin
  *
  * @author Mike Wozniewski
  */




// 
#include <osg/ComputeBoundsVisitor>

#include "asBasicNode.h"
#include "asSceneManager.h"
#include "osgUtil.h"



using namespace std;

//extern asSceneManager *sceneManager;




// ***********************************************************
// constructor:
asBasicNode::asBasicNode (asSceneManager *sceneManager, char *initID) : asReferenced(sceneManager, initID)
{
	nodeType = "asBasicNode";
	this->setName(string(id->s_name) + ".asBasicNode");

	_reportGlobals = false;
	
	mainTransform = new osg::PositionAttitudeTransform();
	mainTransform->setName(string(id->s_name) + ".mainTransform");
	this->addChild(mainTransform.get());

	  
	// When children are attached to this, they get added to the attachNode:
	// NOTE: by changing this, we MUST override the updateNodePath() method!
	attachmentNode = mainTransform.get();

	// We need to set up a callback. This should be on the topmost node, so that during node
	// traversal, we update our parameters before anything is drawn.
	this->setUserData( dynamic_cast<osg::Referenced*>(this) );
	this->setUpdateCallback(new asReferenced_callback);

}

// ***********************************************************
// destructor
asBasicNode::~asBasicNode()
{

}


void asBasicNode::callbackUpdate()
{
	
	dumpGlobals(false); // never force globals here
	
}


void asBasicNode::updateNodePath()
{
	currentNodePath.clear();
	if ((parent!=WORLD_SYMBOL) && (parent!=NULL_SYMBOL))
	{
		osg::ref_ptr<asReferenced> parentNode = parent->s_thing;
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
	
}


// ***********************************************************
// ***********************************************************

void asBasicNode::reportGlobals (int b)
{
	if (this->_reportGlobals != (bool)b)
	{
		this->_reportGlobals = (bool) b;
		BROADCAST(this, "si", "reportGlobals", (int) this->_reportGlobals);	
	}
}


void asBasicNode::setTranslation (float x, float y, float z)
{	
	mainTransform->setPosition(osg::Vec3d(x,y,z));
	
	BROADCAST(this, "sfff", "setTranslation", x, y, z);
}


void asBasicNode::setOrientation (float p, float r, float y)
{
	
	_orientation = osg::Vec3(p, r, y);
	
	osg::Quat q = osg::Quat( osg::DegreesToRadians(p), osg::Vec3d(1,0,0),
							 osg::DegreesToRadians(r), osg::Vec3d(0,1,0),
							 osg::DegreesToRadians(y), osg::Vec3d(0,0,1));
	
	mainTransform->setAttitude(q);
	
	BROADCAST(this, "sfff", "setOrientation", p, r, y);
}

void asBasicNode::move (float x, float y, float z)
{
	osg::Vec3 newPos = mainTransform->getPosition() + ( mainTransform->getAttitude() * osg::Vec3(x,y,z) );
	setTranslation(newPos.x(), newPos.y(), newPos.z());
}

void asBasicNode::rotate (float p, float r, float y)
{
	this->setOrientation(_orientation.x()+p, _orientation.y()+r, _orientation.z()+y);
}

// *****************************************************************************




std::vector<lo_message> asBasicNode::getState ()
{
	// inherit state from base class
	std::vector<lo_message> ret = asReferenced::getState();
	
	lo_message msg;
	osg::Vec3 v;
	

	msg = lo_message_new();
	lo_message_add(msg, "si", "reportGlobals",(int) this->_reportGlobals);
	ret.push_back(msg);

	msg = lo_message_new();
	v = this->getTranslation();
	lo_message_add(msg, "sfff", "setTranslation", v.x(), v.y(), v.z());
	ret.push_back(msg);
	
	msg = lo_message_new();
	v = this->getOrientation();
	lo_message_add(msg, "sfff", "setOrientation", v.x(), v.y(), v.z());
	ret.push_back(msg);
	
	
	//lo_message_free(msg);
	
	return ret;
}

// *****************************************************************************
void asBasicNode::dumpGlobals(bool forced)
{
	// The "forced" parameter means that we do the dump even if there has been
	// no change. This method is called in the updateCallback, which occurs
	// very frequently and should NEVER be forced. The stateDump() method will
	// however force an update of the current global parameters
	
	if (this->_reportGlobals)
	{
		
		// position & rotation: (should we get position from centre of boundingsphere?
		osg::Matrix myMatrix = osg::computeLocalToWorld(this->currentNodePath);
		if ((myMatrix != this->_globalMatrix) || forced)
		{
			this->_globalMatrix = myMatrix;
			osg::Vec3 myPos = myMatrix.getTrans();
			osg::Vec3 myRot = Vec3inDegrees(QuatToEuler(myMatrix.getRotate()));
			
			BROADCAST(this, "sffffff", "global6DOF", myPos.x(), myPos.y(), myPos.z(), myRot.x(), myRot.y(), myRot.z());
		
		
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
		
		
			
		const osg::BoundingSphere& bs = this->getBound();
		if ((bs.radius() != _globalRadius) || forced)
		{
			_globalRadius = bs.radius();
			BROADCAST(this, "sf", "globalRadius", _globalRadius);
		}

	}
}
