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

#include "lodnode.h"
#include "spinapp.h"
#include "scenemanager.h"

namespace spin
{

class SceneManager;

// -----------------------------------------------------------------------------

// constructor:
LODNode::LODNode (SceneManager *sceneManager, const char* initID) : GroupNode(sceneManager, initID)
{
    this->setName(this->getID() + ".LODNode");
    this->setNodeType("LODNode");

    LOD_ = new osg::LOD();
    LOD_->setName(this->getID() + ".LOD");
    
    // We inherit from GroupNode, so we must make sure to attach our osg Switch
	// to the attachmentNode of GroupNode
	this->getAttachmentNode()->addChild(LOD_.get());
	
	// ... and then update the attachmentNode:
	setAttachmentNode(LOD_.get());
}

// destructor
LODNode::~LODNode()
{

}

void LODNode::updateNodePath()
{
	// call GroupNode's method, which will update all the way from the root, and
	// we just need to add the LOD_ node:
	GroupNode::updateNodePath(false);
	currentNodePath_.push_back(LOD_.get());

	// now update NodePaths for all children:
	updateChildNodePaths();
}

// -----------------------------------------------------------------------------

void LODNode::setRange (const char* childID, float min, float max)
{
    ReferencedNode *child = sceneManager_->getNode(childID);

    if (child)
    {
        for (int i=0; i<LOD_->getNumChildren(); i++)
        {
            if (LOD_->getChild(i)==child)
            {
                LOD_->setRange(i, min, max);
            }
        }
        BROADCAST(this, "ssff", "setRange", childID, min, max);
    }
    else
    {
        std::cout << "WARNING: LODNode::setRange couldn't find node: " << childID << std::endl;
    }
}

// -----------------------------------------------------------------------------
std::vector<lo_message> LODNode::getState () const
{
    // inherit state from base class
    std::vector<lo_message> ret = GroupNode::getState();

    lo_message msg;

    for (int i=0; i<LOD_->getNumChildren(); i++)
    {
        ReferencedNode *n = dynamic_cast<ReferencedNode*>(LOD_->getChild(i));
        if (n)
        {
            msg = lo_message_new();
            lo_message_add(msg, "ssff", "setRange", n->getID().c_str(), LOD_->getMinRange(i), LOD_->getMaxRange(i));
            ret.push_back(msg);
        }
    }

    return ret;
}

} // end of namespace spin

