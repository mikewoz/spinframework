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
