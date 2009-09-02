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


#ifndef __ConstraintsNode_H
#define __ASSONSTRAINTS_H


#include "ReferencedNode.h"

#include <osg/Group>
#include <osg/PositionAttitudeTransform> 
#include <osg/NodeVisitor>

#include <string>
#include <vector>
#include <map>


enum constraintMode { NONE, DROP_TO_SURFACE };

/**
 * \brief A node with constrained motion
 * 
 * This node operates much like GroupNode, where children can be attached and
 * positioned somewhere in the scene, but motion is constrained. Typically, the
 * contraints look at properties of the parent node to determine the limitation
 * of movement. For example, the node will only move along the surface of the
 * parent, or along one axis of the parent, etc.
 */
class ConstraintsNode : public ReferencedNode
{

public:
	

	ConstraintsNode(SceneManager *sceneManager, char *initID);
	virtual ~ConstraintsNode();
	
	virtual void callbackUpdate();
	
	/**
	 * IMPORTANT:
	 * subclasses of ReferencedNode are allowed to contain complicated subgraphs,
	 * and can also change their attachmentNode so that children are attached
	 * anywhere in this subgraph. If that is the case, the updateNodePath()
	 * function MUST be overridden, and extra nodes must be manually pushed onto
	 * the currentNodePath.
	 */
	virtual void updateNodePath();
	
	
	void setMode(int m);
	
	void setTranslation (float x, float y, float z);
	void setOrientation (float p, float r, float y);
	void move (float x, float y, float z);
	void rotate (float p, float r, float y);
	
	int getMode() { return (int)_mode; };
	osg::Vec3 getTranslation() { return mainTransform->getPosition(); };
	osg::Vec3 getOrientation() { return _orientation; };
	
	
	/**
	 * For each subclass of ReferencedNode, we override the getState() method to
	 * fill the vector with the correct set of methods for this particular node
	 */
	virtual std::vector<lo_message> getState();
	
	/**
	 * We must include a stateDump() method that simply invokes the base class
	 * method. Simple C++ inheritance is not enough, because osg::Introspection
	 * won't see it.
	 */
	//virtual void stateDump() { ReferencedNode::stateDump(); };

	
	// ***********************************************************
	// data:
	
	osg::ref_ptr<osg::PositionAttitudeTransform> mainTransform;

	
private:
	
	enum constraintMode _mode;
	
	osg::Vec3 _orientation; // store the orientation as it comes in (in degrees)
		
};


	
#endif