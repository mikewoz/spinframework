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

#include <osgUtil/LineSegmentIntersector>
#include <osgUtil/IntersectionVisitor>

#include "ConstraintsNode.h"
#include "SceneManager.h"




using namespace std;



// ***********************************************************
// constructor:
ConstraintsNode::ConstraintsNode (SceneManager *sceneManager, char *initID) : GroupNode(sceneManager, initID)
{
	nodeType = "ConstraintsNode";
	this->setName(string(id->s_name) + ".ConstraintsNode");

	_mode = UNCONSTRAINED;

}

// ***********************************************************
// destructor
ConstraintsNode::~ConstraintsNode()
{

}


void ConstraintsNode::callbackUpdate()
{

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

void ConstraintsNode::setMode (constraintMode m)
{
	if (this->_mode != (int)m)
	{
		this->_mode = m;
		BROADCAST(this, "si", "setMode", (int)_mode);
	}
}

void ConstraintsNode::setTranslation (float x, float y, float z)
{	
	osg::Vec3 v = osg::Vec3(x,y,z);
	
	if (_mode == DROP_TO_PARENT)
	{
		osg::ref_ptr<ReferencedNode> parentNode = dynamic_cast<ReferencedNode*>(parent->s_thing);
		if (parentNode.valid())
		{
			v = computeDropIntersection(parentNode.get(), x, y);
		}
	}
	
	
	GroupNode::setTranslation(v.x(), v.y(), v.z());
}


void ConstraintsNode::translate (float x, float y, float z)
{
	osg::Vec3 v = osg::Vec3(x,y,z);
	
	if (_mode == BOUNCE_OFF_PARENT)
	{
		osg::ref_ptr<ReferencedNode> parentNode = dynamic_cast<ReferencedNode*>(parent->s_thing);
		if (parentNode.valid())
		{
			osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector = 
				new osgUtil::LineSegmentIntersector(this->getCenter(),mainTransform->getPosition()+v);

			osgUtil::IntersectionVisitor iv(intersector.get());

			parentNode->accept(iv);

			if (intersector->containsIntersections())
			{
				osgUtil::LineSegmentIntersector::Intersections intersections;
				osgUtil::LineSegmentIntersector::Intersections::iterator itr;
				
				intersections = intersector->getIntersections();
				
				for (itr = intersections.begin(); itr != intersections.end(); ++itr)
				{
					ReferencedNode *testNode = dynamic_cast<ReferencedNode*>((*itr).nodePath[0]);
					if (testNode)
					{
					
						// TODO
					
					}
				}
			}
		}
		
		
		
		
	}
	
	
	GroupNode::translate(v.x(), v.y(), v.z());
}



// *****************************************************************************
std::vector<lo_message> ConstraintsNode::getState ()
{
	// inherit state from base class
	std::vector<lo_message> ret = GroupNode::getState();
	
	lo_message msg;
	osg::Vec3 v;

	msg = lo_message_new();
	lo_message_add(msg, "si", "setMode", this->getMode());
	ret.push_back(msg);
	
	return ret;
}
