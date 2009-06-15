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


#ifndef FIND_NODE_VISITOR_H
#define FIND_NODE_VISITOR_H

#include <osg/NodeVisitor>
#include <osg/Node>
#include <osg/Geode>
#include <osg/PositionAttitudeTransform>
#include <osg/MatrixTransform>
#include <osg/Switch>
#include <osg/Sequence>

#include <iostream>


/**
 * \brief An OSG NodeVisitor class that allows us to search for a specific node
 *        name/type in the scene graph.
 * 
 * For example, this is used to find the switch & sequence nodes when loading
 * an OSG model in the asModel class, so that we may discover animation features
 */

class findNodeVisitor : public osg::NodeVisitor {

public: 

	findNodeVisitor();

	virtual void apply(osg::Node &searchNode);
	virtual void apply(osg::Geode &searchNode);
	virtual void apply(osg::MatrixTransform &searchNode);
	virtual void apply(osg::PositionAttitudeTransform &searchNode);
	virtual void apply(osg::Switch &searchNode);
	virtual void apply(osg::Sequence &searchNode);
	
	void searchNode(osg::Node* searchFromMe, std::string searchName);
	
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

#endif
