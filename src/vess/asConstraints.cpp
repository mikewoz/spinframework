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
//    La SociŽtŽ des Arts Technologiques (http://www.sat.qc.ca)
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
//  You should have received a copy of the Lesser GNU General Public License
//  along with SPIN Framework. If not, see <http://www.gnu.org/licenses/>.
// -----------------------------------------------------------------------------

#include <osgUtil/LineSegmentIntersector>
#include <osgUtil/IntersectionVisitor>

#include "asConstraints.h"
#include "asSceneManager.h"
#include "osgUtil.h"




using namespace std;



// ***********************************************************
// constructor:
asConstraints::asConstraints (asSceneManager *sceneManager, char *initID) : asReferenced(sceneManager, initID)
{
	nodeType = "asConstraints";
	this->setName(string(id->s_name) + ".asConstraints");

	
	_mode = NONE;

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
asConstraints::~asConstraints()
{

}


void asConstraints::callbackUpdate()
{

}
	

void asConstraints::updateNodePath()
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



// *****************************************************************************
// *****************************************************************************


osg::Vec3 computeDropIntersection(osg::Node* subgraph,float x,float y)
{
    const osg::BoundingSphere& bs = subgraph->getBound();
    float zMax = bs.center().z()+bs.radius();
    float zMin = bs.center().z()-bs.radius();
    
    osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector = 
        new osgUtil::LineSegmentIntersector(osg::Vec3(x,y,zMin),osg::Vec3(x,y,zMax));

    osgUtil::IntersectionVisitor iv(intersector.get());

    subgraph->accept(iv);

    if (intersector->containsIntersections())
    {
        return intersector->getFirstIntersection().getWorldIntersectPoint();
    }

    return osg::Vec3(x,y,0.0f);
}


// *****************************************************************************
// *****************************************************************************

void asConstraints::setMode (int m)
{
	this->_mode = (constraintMode) m;
	BROADCAST(this, "si", "setMode", (int)_mode);
}

void asConstraints::setTranslation (float x, float y, float z)
{	
	osg::Vec3 v;
	
	if (this->_mode == DROP_TO_SURFACE)
	{
		osg::ref_ptr<asReferenced> parentNode = parent->s_thing;
		if (parentNode.valid())
		{
			v = computeDropIntersection(parentNode.get(), x, y);
		}
	}
		
	else {
		v = osg::Vec3(x,y,z);
	}
	
	mainTransform->setPosition(v);
	BROADCAST(this, "sfff", "setTranslation", v.x(), v.y(), v.z());
}


void asConstraints::setOrientation (float p, float r, float y)
{
	
	_orientation = osg::Vec3(p, r, y);
	
	osg::Quat q = osg::Quat( osg::DegreesToRadians(p), osg::Vec3d(1,0,0),
							 osg::DegreesToRadians(r), osg::Vec3d(0,1,0),
							 osg::DegreesToRadians(y), osg::Vec3d(0,0,1));
	
	mainTransform->setAttitude(q);
	
	BROADCAST(this, "sfff", "setOrientation", p, r, y);
}

void asConstraints::move (float x, float y, float z)
{
	osg::Vec3 newPos = mainTransform->getPosition() + ( mainTransform->getAttitude() * osg::Vec3(x,y,z) );
	setTranslation(newPos.x(), newPos.y(), newPos.z());
}

void asConstraints::rotate (float p, float r, float y)
{
	this->setOrientation(_orientation.x()+p, _orientation.y()+r, _orientation.z()+y);
}

// *****************************************************************************




std::vector<lo_message> asConstraints::getState ()
{
	// inherit state from base class
	std::vector<lo_message> ret = asReferenced::getState();
	
	lo_message msg;
	osg::Vec3 v;
	

	msg = lo_message_new();
	lo_message_add(msg, "si", "setMode", this->getMode());
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
