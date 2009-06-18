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
//  You should have received a copy of the Lesser GNU General Public License
//  along with SPIN Framework. If not, see <http://www.gnu.org/licenses/>.
// -----------------------------------------------------------------------------

#include <iostream>
#include <algorithm>

#include <osg/Timer>
#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>

#include "asRayIntersector.h"
#include "asSceneManager.h"
#include "asRay.h"


using namespace std;

//extern asSceneManager *sceneManager;


// *****************************************************************************
// constructor:
asRayIntersector::asRayIntersector (asSceneManager *sceneManager, char *initID) : asReferenced(sceneManager, initID)
{
	
	this->setName(string(id->s_name) + ".asRayIntersector");
	nodeType = "asRayIntersector";
	
	intersectList.clear();
	
	// We need to set up a callback, that computes intersections during scene
	// graph traversal. This should be on the topmost node, so that during node
	// traversal, we update our parameters before anything is drawn.
	this->setUserData( dynamic_cast<osg::Referenced*>(this) );
	this->setUpdateCallback(new asReferenced_callback);
	
	this->setNodeMask(STATSDATA_NODE_MASK); // nodemask info in asGlobals.h
	
}

// *****************************************************************************
// destructor
asRayIntersector::~asRayIntersector()
{

}


// *****************************************************************************



void asRayIntersector::callbackUpdate()
{
	int i;

	
	// get parentNode:
	osg::ref_ptr<asRay> parentNode = dynamic_cast<asRay*>(this->parent->s_thing);
	
	if (!parentNode.valid())
	{
		std::cout << "Error: asRayIntersector [" << this->id->s_name << "] must be attached to an asRay node." << std::endl;
		return;
	}

	//osg::Timer_t startTick = osg::Timer::instance()->tick();

	// get line segment start and end points:
	osg::Matrix myMatrix = osg::computeLocalToWorld(this->currentNodePath);
	osg::Vec3 start = myMatrix.getTrans();
	osg::Vec3 end = start + ( myMatrix.getRotate() * osg::Vec3(0.0,parentNode->getLength(),0.0) );

	// TODO: only proceed if myMatrix has changed since last time ??
	
	// create an intersector and create an intersectorVisitor.
	osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector = new osgUtil::LineSegmentIntersector(start, end);
	osgUtil::IntersectionVisitor intersectVisitor( intersector.get() );
	
	// make sure to provide a nodemask so that only geometric nodes are checked
	// for intersections:
	intersectVisitor.setTraversalMask(GEOMETRIC_NODE_MASK);
	
	// start the visitor:
	sceneManager->worldNode->accept(intersectVisitor);

	//osg::Timer_t endTick = osg::Timer::instance()->tick();
	//std::cout<<"Intersection completed in "<<osg::Timer::instance()->delta_s(startTick,endTick)<<std::endl;

	//intersectList.clear();
	
	std::vector<t_symbol*> newIntersectList;
	vector<t_symbol*>::iterator iter;
	
	if ( intersector->containsIntersections() )
	{
		osg::ref_ptr<asReferenced> testNode;
		
		osgUtil::LineSegmentIntersector::Intersections& intersections = intersector->getIntersections();
		osgUtil::LineSegmentIntersector::Intersections::iterator itr;

		for (itr = intersections.begin(); itr != intersections.end(); ++itr)
		{
			const osgUtil::LineSegmentIntersector::Intersection& intersection = *itr;
			for (i=intersection.nodePath.size()-1; i>=0; i--)
			{
				testNode = dynamic_cast<asReferenced*>(intersection.nodePath[i]);
				if ((testNode.valid()) && (testNode->nodeType != "ray"))
				{
					// only add the hit if not already in the list:
					iter = std::find( newIntersectList.begin(), newIntersectList.end(), testNode->id );
					if ( iter == newIntersectList.end() ) newIntersectList.push_back(testNode->id);
					break;
				}
				//std::cout << " > " << intersection.nodePath[i]->getName();
			}
			//std::cout << std::endl;

			/*
			std::cout<<"  ratio "<<intersection.ratio<<std::endl;
			std::cout<<"  point "<<intersection.localIntersectionPoint<<std::endl;
			std::cout<<"  normal "<<intersection.localIntersectionNormal<<std::endl;
			std::cout<<"  indices "<<intersection.indexList.size()<<std::endl;
			std::cout<<"  primitiveIndex "<<intersection.primitiveIndex<<std::endl;
			*/


		}
		
		/*
		std::cout << " OLD intersections:";
		for (i=0; i<intersectList.size(); i++) std::cout << " " << intersectList[i]->s_name;
		std::cout << std::endl;
		
		std::cout << " NEW intersections:";
		for (i=0; i<newIntersectList.size(); i++) std::cout << " " << newIntersectList[i]->s_name;
		std::cout << std::endl;
		*/
		
	}
		
	// Check to see if the intersectList has changed from the previous one:
	bool intersectChange = false;
	if (intersectList.size()==newIntersectList.size())
	{
		if (!std::equal(intersectList.begin(), intersectList.end(), newIntersectList.begin())) intersectChange = true;
	} else intersectChange = true;
			
	// if the intersectList has changed, then we replace our old one and
	// broadcast the new list:
	if (intersectChange)
	{
		intersectList.clear();
		intersectList.resize(newIntersectList.size());
		std::copy(newIntersectList.begin(), newIntersectList.end(), intersectList.begin());
			
		lo_message msg = lo_message_new();
		lo_message_add_string(msg, "intersectsWith");
		if (intersectList.size())
		{
			for (i=0; i<intersectList.size(); i++) lo_message_add_string(msg, (char*)intersectList[i]->s_name);
		} else lo_message_add_string(msg, "NULL");
		
		sceneManager->sendNodeMessage(this->id, msg);
		
	}
	
}

/*
void asRayIntersector::getIntersections()
{

	osg::BoundingSphere bs = scene->getBound();
	
	osg::Timer_t startTick = osg::Timer::instance()->tick();

	osg::Vec3d start = bs.center() + osg::Vec3d(0.0,bs.radius(),0.0);
	osg::Vec3d end = bs.center() - osg::Vec3d(0.0, bs.radius(),0.0);


	osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector = new osgUtil::LineSegmentIntersector(start, end);

	osgUtil::IntersectionVisitor intersectVisitor( intersector.get(), new MyReadCallback );

	scene->accept(intersectVisitor);

	osg::Timer_t endTick = osg::Timer::instance()->tick();

	std::cout<<"Intersection completed in "<<osg::Timer::instance()->delta_s(startTick,endTick)<<std::endl;

	if ( intersector->containsIntersections() )
	{
		osgUtil::LineSegmentIntersector::Intersections& intersections = intersector->getIntersections();
		osgUtil::LineSegmentIntersector::Intersections::iterator itr;
		for (itr = intersections.begin(); itr != intersections.end(); ++itr)
		{
			const osgUtil::LineSegmentIntersector::Intersection& intersection = *itr;
			std::cout<<"  ratio "<<intersection.ratio<<std::endl;
			std::cout<<"  point "<<intersection.localIntersectionPoint<<std::endl;
			std::cout<<"  normal "<<intersection.localIntersectionNormal<<std::endl;
			std::cout<<"  indices "<<intersection.indexList.size()<<std::endl;
			std::cout<<"  primitiveIndex "<<intersection.primitiveIndex<<std::endl;
			std::cout<<std::endl;
		}
	}
	                                 
}

*/

std::vector<lo_message> asRayIntersector::getState ()
{
	// inherit state from base class
	std::vector<lo_message> ret = asReferenced::getState();
	
	return ret;
}
