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
