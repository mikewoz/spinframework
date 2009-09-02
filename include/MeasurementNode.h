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

#ifndef MeasurementNode_H_
#define MeasurementNode_H_

#include "ReferencedNode.h"

/**
 * \brief A node that can be used to measure relations to another node.
 * 
 * 
 * A targetNode must be specified, and then measurements (such as distance,
 * incidence of orientation, etc.) are computed in the callback and can be
 * reported with different levels of detail.
 */
class MeasurementNode : public ReferencedNode
{

public:

	MeasurementNode(SceneManager *sceneManager, char *initID);
	virtual ~MeasurementNode();
	
	virtual void callbackUpdate();

	void setTarget (char *id);
	void setReportingLevel (int level);
	
	const char* getTarget();
	int getReportingLevel() { return this->reportingLevel; }
	
	
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
	
	
private:
	
	osg::ref_ptr<ReferencedNode> targetNode;
	int reportingLevel;
	
	
	osg::Matrix thisMatrix, targetMatrix;
	
};



#endif
