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

#ifndef nodeVisitors_H
#define nodeVisitors_H

#include <osg/NodeVisitor>
#include <osg/Node>
#include <osg/Geode>
#include <osg/PositionAttitudeTransform>
#include <osg/MatrixTransform>
#include <osg/Switch>
#include <osg/Sequence>

#include <iostream>

#include "ReferencedNode.h"
#include "UserNode.h"


/**
 * \brief An OSG NodeVisitor class that allows us to search for a specific node
 *        name/type in the scene graph.
 * 
 * For example, this is used to find the switch & sequence nodes when loading
 * an OSG model in the ModelNode class, so that we may discover animation features
 */

class SearchVisitor : public osg::NodeVisitor {

public: 

	SearchVisitor() : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN)
	{
		// Flag all osg object as NULL
		GeodeNode = NULL;
		MTNode = NULL;
		PATNode = NULL;
		SwitchNode = NULL;
		SequenceNode = NULL;
	}

	virtual void apply(osg::Node &searchNode) 
	{
		traverse(searchNode);
	}
	
	virtual void apply(osg::Geode &searchNode)
	{
		osg::ref_ptr<osg::Geode> n = dynamic_cast<osg::Geode*> (&searchNode);
		if (n.valid())
		{
			if (searchForName == searchNode.getName()) GeodeNode = n;
		}
		traverse(searchNode);
	}
	virtual void apply(osg::MatrixTransform &searchNode)
	{
		osg::ref_ptr<osg::MatrixTransform> n = dynamic_cast<osg::MatrixTransform*> (&searchNode);
		if (n.valid())
		{
			if (searchForName == searchNode.getName()) MTNode = n;
		}
		traverse(searchNode);
	}
	virtual void apply(osg::PositionAttitudeTransform &searchNode)
	{
		osg::ref_ptr<osg::PositionAttitudeTransform> n = dynamic_cast<osg::PositionAttitudeTransform*> (&searchNode);
		if (n.valid())
		{
			if (searchForName == searchNode.getName()) PATNode = n;
		}
		traverse(searchNode);
	}
	virtual void apply(osg::Switch &searchNode)
	{
		osg::ref_ptr<osg::Switch> n = dynamic_cast<osg::Switch*> (&searchNode);
		if (n.valid())
		{
			if (searchForName == searchNode.getName()) SwitchNode = n;
		}
		traverse(searchNode);
	}
	virtual void apply(osg::Sequence &searchNode)
	{
		osg::ref_ptr<osg::Sequence> n = dynamic_cast<osg::Sequence*> (&searchNode);
		if (n.valid())
		{
			if (searchForName == searchNode.getName()) SequenceNode = n;
		}
		traverse(searchNode);
	}

	// search for node with given name starting at the search node
	void searchNode(osg::Node* searchFromMe, std::string searchName)
	{
		GeodeNode = NULL;
		MTNode = NULL;
		PATNode = NULL;
		SwitchNode = NULL;
		SequenceNode = NULL;

		searchForName = searchName;
		searchFromMe->accept(*this);
	}

	osg::ref_ptr<osg::MatrixTransform> getMT() { return MTNode; }
	osg::ref_ptr<osg::Geode> getGeode() { return GeodeNode; }
	osg::ref_ptr<osg::PositionAttitudeTransform> getPAT() { return PATNode; }
	osg::ref_ptr<osg::Switch> getSwitchNode() { return SwitchNode; }
	osg::ref_ptr<osg::Sequence> getSequenceNode() { return SequenceNode; }
	
private: 

	std::string searchForName;
	
	osg::ref_ptr<osg::Geode> GeodeNode;
	osg::ref_ptr<osg::MatrixTransform> MTNode;
	osg::ref_ptr<osg::PositionAttitudeTransform> PATNode;
	osg::ref_ptr<osg::Switch> SwitchNode;
	osg::ref_ptr<osg::Sequence> SequenceNode;
	
};




/**
 * \brief An OSG NodeVisitor class that pretty prints the scene graph
 */
class DebugVisitor : public osg::NodeVisitor
{
	public:
		
		DebugVisitor() : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN) {}

		virtual void apply(osg::Node &node)
		{
			osg::notify(osg::NOTICE) << leadingSpaces(getNodePath().size()) << "NODE:  " << node.getName() << std::endl;
			traverse(node);
		}
		virtual void apply(osg::PositionAttitudeTransform &node)
		{
			osg::notify(osg::NOTICE) << leadingSpaces(getNodePath().size()) << "PAT:   " << node.getName() << "  (" << node.getNumChildren () << " children)" << std::endl;
			traverse(node);
		}
		virtual void apply(osg::Geode &node)
		{
			osg::notify(osg::NOTICE) << leadingSpaces(getNodePath().size()) << "GEODE: " << node.getName() << std::endl;
			traverse(node);
		}
		virtual void apply(osg::Group &node)
		{
			ReferencedNode *n;
						
			if (n=dynamic_cast<ReferencedNode*>(&node)) {
				osg::notify(osg::NOTICE) << leadingSpaces(getNodePath().size()) << "SPIN NODE: type=" << n->nodeType << ", id=" << n->id << "  (" << node.getNumChildren () << " children)" << std::endl;
			} else {
				osg::notify(osg::NOTICE) << leadingSpaces(getNodePath().size()) << "GROUP: " << node.getName() << "  (" << node.getNumChildren () << " children)" << std::endl;
			}
			traverse(node);
		}
};

/**
 * \brief A NodeVisitor class that invokes the update callback in every node
 */
class UpdateSceneVisitor : public osg::NodeVisitor
{
	public:

		UpdateSceneVisitor() : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN) {}

		virtual void apply(osg::Node &node)
		{
			traverse(node);
		}
		virtual void apply(osg::Group &node)
		{
			ReferencedNode *n;
			if (n=dynamic_cast<ReferencedNode*>(&node)) {
				n->callbackUpdate();
			}
			traverse(node);
		}
};








#endif
