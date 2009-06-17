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


#include "findNodeVisitor.h"

// set the traversal mode to TRAVERSE_ALL_CHILDREN
findNodeVisitor::findNodeVisitor() : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN)
{
	// Flag all osg object as NULL
	GeodeNode = NULL;
	MTNode = NULL;
	PATNode = NULL;
	SwitchNode = NULL;
	SequenceNode = NULL;
}

void findNodeVisitor::apply(osg::Node &searchNode) 
{
	traverse(searchNode);
}

void findNodeVisitor::apply(osg::Geode &searchNode)
{
	osg::ref_ptr<osg::Geode> searchGeodeNode = dynamic_cast<osg::Geode*> (&searchNode);
	
	if (searchGeodeNode.valid()) // make sure valid MatrixXform node
	{
		if (searchForName == searchNode.getName()) GeodeNode = searchGeodeNode;
	}
	
	//apply ( (osg::Node&) searchNode);
	traverse(searchNode);
}

void findNodeVisitor::apply(osg::MatrixTransform &searchNode)
{
	osg::ref_ptr<osg::MatrixTransform> searchMformNode = dynamic_cast<osg::MatrixTransform*> (&searchNode);
	
	if (searchMformNode.valid()) // make sure valid MatrixXform node
	{
		if (searchForName == searchNode.getName()) MTNode = searchMformNode;
	}
	
	//apply ( (osg::Node&) searchNode);
	traverse(searchNode);
}

void findNodeVisitor::apply(osg::PositionAttitudeTransform &searchNode)
{
	osg::ref_ptr<osg::PositionAttitudeTransform> searchPATformNode = dynamic_cast<osg::PositionAttitudeTransform*> (&searchNode);
	
	if (searchPATformNode.valid())
	{
		if (searchForName == searchNode.getName()) PATNode = searchPATformNode;
	}
	
	//apply ( (osg::Node&) searchNode);
	traverse(searchNode);
}

void findNodeVisitor::apply(osg::Switch &searchNode)
{
	osg::ref_ptr<osg::Switch> searchSwitchNode = dynamic_cast<osg::Switch*> (&searchNode);
	
	if (searchSwitchNode.valid())
	{
		if (searchForName == searchNode.getName()) SwitchNode = searchSwitchNode;
	}
	
	//apply ( (osg::Node&) searchNode);
	traverse(searchNode);
}

void findNodeVisitor::apply(osg::Sequence &searchNode)
{
	osg::ref_ptr<osg::Sequence> searchSequenceNode = dynamic_cast<osg::Sequence*> (&searchNode);
	
	if (searchSequenceNode.valid())
	{
		if (searchForName == searchNode.getName()) SequenceNode = searchSequenceNode;
	}
	
	//apply ( (osg::Node&) searchNode);
	traverse(searchNode);
}

// search for node with given name starting at the search node
void findNodeVisitor::searchNode(osg::Node* searchFromMe, std::string searchName)
{
	GeodeNode = NULL;
	MTNode = NULL;
	PATNode = NULL;
	SwitchNode = NULL;
	SequenceNode = NULL;

	searchForName = searchName;
	searchFromMe->accept(*this);
}
