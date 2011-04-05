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

#include "SwitchNode.h"
#include <osg/ShapeDrawable>
#include <osgUtil/Optimizer>
#include <osg/Geometry>
#include <osgUtil/SmoothingVisitor>
#include <osg/Switch>
#include <lo/lo_types.h>

#include "SceneManager.h"
#include "spinApp.h"
#include "spinBaseContext.h"
#include "osgUtil.h"

//extern SceneManager *sceneManager;

namespace spin
{

// *****************************************************************************
// constructor:
SwitchNode::SwitchNode (SceneManager *sceneManager, char *initID) : GroupNode(sceneManager, initID)
{
    using std::string;
	this->setName(string(id->s_name) + ".SwitchNode");
	nodeType = "SwitchNode";

	switcher = new osg::Switch();
	switcher->setName(string(id->s_name) + ".switcher");

	std::cout << "creating switch node. attaching to " << this->getAttachmentNode()->getName() << std::endl;

	// We inherit from GroupNode, so we must make sure to attach our osg Switch
	// to the attachmentNode of GroupNode
	this->getAttachmentNode()->addChild(switcher.get());
	
	// ... and then update the attachmentNode:
	setAttachmentNode(switcher.get());
}

// *****************************************************************************
// destructor
SwitchNode::~SwitchNode()
{

}

void SwitchNode::updateNodePath()
{
	// call GroupNode's method, which will update all the way from the root, and
	// we just need to add the Switch node:
	GroupNode::updateNodePath(false);
	currentNodePath.push_back(switcher.get());

	// now update NodePaths for all children:
	updateChildNodePaths();
}


// *****************************************************************************
void SwitchNode::setEnabled (const char* id, int enabled)
{
	for (unsigned int i=0; i<switcher->getNumChildren(); i++)
	{
		osg::ref_ptr<ReferencedNode> n = dynamic_cast<ReferencedNode*>(switcher->getChild(i));
		if (n.valid() && (n->id==gensym(id)))
		{
			// found the node in question
			switcher->setValue(i, (bool)enabled);
			BROADCAST(this, "ssi", "setEnabled", id, (int)switcher->getValue(i));
			break;
		}
	}
}

void SwitchNode::setAll(int enabled)
{
	if (enabled)
		switcher->setAllChildrenOn();
	else
		switcher->setAllChildrenOff();
	
	for (unsigned int i=0; i<switcher->getNumChildren(); i++)
	{
		osg::ref_ptr<ReferencedNode> n = dynamic_cast<ReferencedNode*>(switcher->getChild(i));
		if (n.valid())
		{
			BROADCAST(this, "ssi", "setEnabled", n->id->s_name, (int)switcher->getValue(i));
		}
	}
}

// *****************************************************************************

std::vector<lo_message> SwitchNode::getState () const
{
	// inherit state from base class
	std::vector<lo_message> ret = GroupNode::getState();

	lo_message msg;
	for (unsigned int i=0; i<switcher->getNumChildren(); i++)
	{
		osg::ref_ptr<ReferencedNode> n = dynamic_cast<ReferencedNode*>(switcher->getChild(i));
		if (n.valid())
		{
			msg = lo_message_new();
			lo_message_add(msg, "ssi", "setEnabled", n->id->s_name, (int)switcher->getValue(i));
			ret.push_back(msg);
		}
	}
	return ret;
}

} // end of namespace spin
