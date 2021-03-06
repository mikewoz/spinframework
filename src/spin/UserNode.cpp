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

#include "usernode.h"
#include "scenemanager.h"
#include "spinapp.h"
#include "osgutil.h"
#include "spinbasecontext.h"

namespace spin
{

// *****************************************************************************
// constructor:
UserNode::UserNode (SceneManager *sceneManager, const char* initID) : ConstraintsNode(sceneManager, initID)
{
    this->setNodeType("UserNode");
    this->setName(this->getID() + ".UserNode");

    description_ = std::string(initID);
	computationMode_ = CLIENT_SIDE; 
	velocityMode_ = MOVE;	
	cameraOffsetNode_ = new osg::PositionAttitudeTransform();
	cameraOffsetNode_->setName(this->getID() + ".cameraOffset");
	cameraAttachmentNode_ = new osg::PositionAttitudeTransform();
	cameraAttachmentNode_->setName(this->getID() + ".cameraAttachmentNode");

    // should we attach ourselves to GroupNode's mainTransform or clipNode?
	getAttachmentNode()->addChild(cameraOffsetNode_.get());
	cameraOffsetNode_->addChild(cameraAttachmentNode_.get());
    
    //setAttachmentNode(cameraAttachmentNode_.get());

    	setReportMode(GroupNode::GLOBAL_6DOF);
    
	homePos_ = osg::Vec3(0.0, -5.0, 0.5);
	goHome();

    ping_ = false;
}

// destructor
UserNode::~UserNode()
{
    //std::cout << "Destroying UserNode: " << getID() << std::endl;
}

void UserNode::callbackUpdate(osg::NodeVisitor* nv)
{
	ConstraintsNode::callbackUpdate(nv);

	if (ping_)
	{
		osg::Timer_t t = osg::Timer::instance()->tick();
		if (osg::Timer::instance()->delta_s(lastPing_,t) > 30) // every 30 seconds
		{
			// this user stopped pinging, so we should remove him from the
			// subgraph.

			//uncomment when ready:
			this->scheduleForDeletion_ = true;
		}
	}

}

void UserNode::updateNodePath(bool updateChildren)
{
    ConstraintsNode::updateNodePath(false);

    /*
    currentNodePath_.push_back(cameraOffsetNode_.get());
    currentNodePath_.push_back(cameraAttachmentNode_.get());
    */
    
    // now update NodePaths for all children:
    if (updateChildren) updateChildNodePaths();

    this->nodepathUpdate = true;
}

// *****************************************************************************

void UserNode::setDescription (const char *newvalue)
{
    using std::string;
    description_ = string(newvalue);
    BROADCAST(this, "ss", "setDescription", getDescription());
}


void UserNode::setHome(float x, float y, float z, float pitch, float roll, float yaw)
{
	homePos_ = osg::Vec3(x,y,z);
	homeRot_ = osg::Vec3(pitch,roll,yaw);
}

void UserNode::goHome()
{
	setTranslation(homePos_.x(), homePos_.y(), homePos_.z());
	setOrientation(homeRot_.x(), homeRot_.y(), homeRot_.z());
}

void UserNode::ping()
{
	lastPing_ = osg::Timer::instance()->tick();
	ping_ = true;
}

void UserNode::setCameraOffset(float x, float y, float z)
{
    osg::Vec3 newOffset = osg::Vec3(x,y,z);

    if (newOffset != getCameraOffset())
    {
        cameraOffsetNode_->setPosition(newOffset);
        BROADCAST(this, "sfff", "setCameraOffset", x, y, z);
    }
}

void UserNode::setCameraOrientationQuat (float x, float y, float z, float w)
{
	osg::Quat newQuat = osg::Quat(x,y,z,w);

    if (newQuat != cameraAttachmentNode_->getAttitude())
    {
        cameraAttachmentNode_->setAttitude(newQuat);
        eulers_ = Vec3inDegrees(QuatToEuler(newQuat));

        BROADCAST(this, "sffff", "setCameraOrientationQuat", x, y, z, w);
    }

}

void UserNode::setCameraOrientation (float pitch, float roll, float yaw)
{
    osg::Vec3 newOrientation = osg::Vec3(pitch, roll, yaw);

    if (newOrientation != getOrientation())
    {
        eulers_ = newOrientation;
        osg::Quat q = osg::Quat( osg::DegreesToRadians(pitch), osg::Vec3d(1,0,0),
                                 osg::DegreesToRadians(roll), osg::Vec3d(0,1,0),
                                 osg::DegreesToRadians(yaw), osg::Vec3d(0,0,1));
        cameraAttachmentNode_->setAttitude(q);
        BROADCAST(this, "sfff", "setCameraOrientation", pitch, roll, yaw);
    }
}

// *****************************************************************************

std::vector<lo_message> UserNode::getState () const
{
	// inherit state from base class
	std::vector<lo_message> ret = ConstraintsNode::getState();

	lo_message msg;
    osg::Vec3 v;

	msg = lo_message_new();
	lo_message_add(msg, "ss", "setDescription", getDescription());
	ret.push_back(msg);

    msg = lo_message_new();
    v = getCameraOffset();
    lo_message_add(msg, "sfff", "setCameraOffset", v.x(), v.y(), v.z());
    ret.push_back(msg);

    msg = lo_message_new();
    v = getCameraOrientation();
    lo_message_add(msg, "sfff", "setCameraOrientation", v.x(), v.y(), v.z());
    ret.push_back(msg);

	return ret;
}

} // end of namespace spin
