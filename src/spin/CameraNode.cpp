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


#include <osg/Group>


#include "CameraNode.h"
#include "SceneManager.h"

using namespace std;



// ===================================================================
// constructor:
caneraNode::CameraNode (SceneManager *sceneManager, char *initID) : ReferencedNode(sceneManager, initID)
{
    this->setName(string(id->s_name) + ".CameraNode");
    nodeType = "CameraNode";

    cameraTransform = new osg::PositionAttitudeTransform();
    cameraTransform->setName(string(id->s_name) + ".cameraTransform");
    this->addChild(cameraTransform.get());

    modelName = "NULL";

    // When children are attached to this, they get added to the attachNode:
    // NOTE: by changing this, we MUST override the updateNodePath() method!
    attachmentNode = cameraTransform.get();

}

// ===================================================================
// destructor
CameraNode::~CameraNode()
{

}

// *****************************************************************************
// IMPORTANT:
// subclasses of ReferencedNode are allowed to contain complicated subgraphs, and
// can also change their attachmentNode so that children are attached anywhere
// in this subgraph. If that is the case, the updateNodePath() function MUST be
// overridden, and extra nodes must be manually pushed onto the currentNodePath.

void CameraNode::updateNodePath()
{

    ReferencedNode::callbackUpdate();

    currentNodePath.clear();
    if ((parent!=WORLD_SYMBOL) && (parent!=NULL_SYMBOL))
    {
        osg::ref_ptr<ReferencedNode> parentNode = parent->s_thing;
        if (parentNode.valid())
        {
            currentNodePath = parentNode->currentNodePath;
        }
    }

    // here, the nodePath includes the base osg::group, PLUS the modelTransform
    currentNodePath.push_back(this);
    currentNodePath.push_back(cameraTransform.get());

    // now update NodePaths for all children:
    updateChildNodePaths();

}


// ===================================================================
// ======================== SET METHODS: =============================
// ===================================================================

void CameraNode::setTranslation (float x, float y, float z)
{
    cameraTransform->setPosition(osg::Vec3(x,y,z));
    BROADCAST(this, "sfff", "setTranslation", x, y, z);
}

void CameraNode::setOrientation (float p, float r, float y)
{
    _orientation = osg::Vec3(p, r, y);

    osg::Quat q = osg::Quat( osg::DegreesToRadians(p), osg::Vec3d(1,0,0),
                             osg::DegreesToRadians(r), osg::Vec3d(0,1,0),
                             osg::DegreesToRadians(y), osg::Vec3d(0,0,1));

    cameraTransform->setAttitude(q);
    BROADCAST(this, "sfff", "setOrientation", p, r, y);
}




std::vector<lo_message> ModelNode::getState ()
{

    // inherit state from base class
    std::vector<lo_message> ret = ReferencedNode::getState();

    lo_message msg;
    osg::Vec3 v;


    msg = lo_message_new();
    v = this->getTranslation();
    lo_message_add(msg, "sfff", "setTranslation", v.x(), v.y(), v.z());
    ret.push_back(msg);

    msg = lo_message_new();
    v = this->getOrientation();
    lo_message_add(msg, "sfff", "setOrientation", v.x(), v.y(), v.z());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "sffff", "setViewport", _viewport.x(), _viewport.y(), _viewport.z(), _viewport.w());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "sffff", "setClearColor", _clearColor.x(), _clearColor.y(), _clearColor.z(), _clearColor.w());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "sffffff", "setFrustum", _frustumXY.x(), _frustumXY.y(), _frustumXY.z(), _frustumXY.w(), _frustumNear, _frustumFar);
    ret.push_back(msg);


    return ret;
}
