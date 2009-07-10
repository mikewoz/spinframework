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

#ifndef ASMANIPULATOR_H_
#define ASMANIPULATOR_H_

#include <osg/MatrixTransform>
#include <osgManipulator/Dragger>
#include <osgUtil/LineSegmentIntersector>

#include "asGlobals.h"
#include "asReferenced.h"


// dummy class:
class asPointerActionAdapter : public osgGA::GUIActionAdapter{
	void requestRedraw(void){};
	void requestContinuousUpdate(bool){};
	void requestWarpPointer(float,float) {};
};

/**
 * \brief An interaction node that reports intersections with other nodes in the
 *        scene
 * 
 * This node must be attached to an asRay node, and reports a list of all nodes
 * with which the ray is intersecting (in order of closest to furthest). There
 * is also a grabber manipulator that allows the first node to be "grabbed" and
 * moved around.
 */

class asPointer : public asReferenced
{

public:

	asPointer(asSceneManager *sceneManager, char *initID);
	virtual ~asPointer();
	
	virtual void callbackUpdate();
	
	
	void enableDragger();
	void disableDragger();

	asReferenced *getNodeFromIntersections();
	//void computeRT(t_symbol *src, t_symbol *dst, osg::Vec3 &R, osg::Vec3 &T);

	void setType (char *s);
	void highlight (int b);
	void manipulate (int b);
	
	const char* getType() { return draggerType.c_str(); }
	int getHighlight() { return (int) dragger.valid(); }
	int getManipulate() { return (int) doManipulation; }
	
	// grab stuff:
	void grab (int b);
	void pull (float f);
	int getGrab() { return (int) grabbedNode.valid(); }
		

	
	/**
	 * For each subclass of asReferenced, we override the getState() method to
	 * fill the vector with the correct set of methods for this particular node
	 */
	virtual std::vector<lo_message> getState();
	
	/**
	 * We must include a stateDump() method that simply invokes the base class
	 * method. Simple C++ inheritance is not enough, because osg::Introspection
	 * won't see it.
	 */
	//virtual void stateDump() { asReferenced::stateDump(); };
	
	
private:
	
	// intersector stuff:
	osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector;
	std::vector<t_symbol*> intersectList;
	std::vector<osg::Vec3> intersectListOffsets;
	std::vector<osgUtil::LineSegmentIntersector::Intersection> intersectData;
	
	// grabber stuff:
	osg::ref_ptr<asReferenced> grabbedNode;
	osg::Vec3 grabbedOffset;
	t_symbol *previousParent;
	
	// dragger stuff:
	std::string draggerType;
	bool doManipulation;
	osg::ref_ptr<asReferenced> targetNode;
	
	osg::Matrix origMatrix;
	osg::Matrix previousMatrix;
	
	osg::ref_ptr<osgManipulator::Dragger> dragger;
	osg::ref_ptr<osgManipulator::Selection> selection;
	osg::ref_ptr<osgManipulator::CommandManager> cmdMgr;
	
	osg::ref_ptr<osgGA::GUIEventAdapter> ea;
	asPointerActionAdapter aa;
	osgManipulator::PointerInfo _pointer;

	
};

#endif
