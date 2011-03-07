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

#include "AttractorNode.h"
#include "spinApp.h"
#include "spinBaseContext.h"
#include "SceneManager.h"
#include "osgUtil.h"

using namespace std;




// ***********************************************************
// constructor:
AttractorNode::AttractorNode (SceneManager *sceneManager, char *initID) : GroupNode(sceneManager, initID)
{
    nodeType = "AttractorNode";
    this->setName(string(id->s_name) + ".AttractorNode");

    distanceDecay_ = 2.0; // exponential force
    angularDecay_ = 0.0; // omni-directional force
    force_ = 0.0; // zero at start
    mode_ = AttractorNode::EXTRINSIC; // does not change target's velocity

    // keep a timer for update calculation:
    lastTick_ = osg::Timer::instance()->tick();
}

// ***********************************************************
// destructor
AttractorNode::~AttractorNode()
{

}

#define EPSILON 0.0001

void AttractorNode::callbackUpdate()
{
	GroupNode::callbackUpdate();

    if ( spinApp::Instance().getContext()->isServer() )
    {
    	// only update when dt is at least 0.05s (ie 20hz):
        osg::Timer_t tick = osg::Timer::instance()->tick();
        float dt = osg::Timer::instance()->delta_s(lastTick_,tick);
        //if (dt > 0.05) // 20 hz
        if (dt > 0.0333) // 30hz
        {
        	if (force_ != 0)
        	{
        		osg::Matrixd thisMat = osg::computeLocalToWorld(this->currentNodePath);

				// get direction of force by converting the AttractorNode's
				// orientation to world coords:
				osg::Quat thisQuat = thisMat.getRotate();
				osg::Vec3 thisDir = thisQuat * osg::Y_AXIS;

				// now cycle through each target and apply the force:
				targetVector::iterator t;
				for (t = targets_.begin(); t != targets_.end(); t++)
				{
					osg::Quat forceOrientation;

					osg::Matrixd targetMat = osg::computeLocalToWorld((*t)->currentNodePath);
					osg::Quat targetQuat = targetMat.getRotate();
					osg::Vec3 targetDir = targetQuat * osg::Y_AXIS;

					osg::Vec3 connection_vector = targetMat.getTrans() - thisMat.getTrans();


					float incidence = AngleBetweenVectors(thisDir, connection_vector);
					float distanceScalar = 1 / (1.0 + pow(connection_vector.length(), distanceDecay_));
					double angularScalar = 1 / (1.0 + pow(incidence, angularDecay_));




					// If the target and attractor are at the same location, the
					// force direction is undefined. So, in that case, we choose
					// the orientation of the attractor as the direction

					osg::Vec3 delta;

					if (connection_vector.length() == 0)
					{
						//forceOrientation = thisQuat;
						if (force_ > 0) delta = thisQuat * osg::Y_AXIS;
						else delta = thisQuat * -osg::Y_AXIS;
					}
					else {
						//forceOrientation.makeRotate(targetMat.getRotate()*osg::Y_AXIS, connection_vector);
						if (force_ > 0) delta = connection_vector;
						else delta = -connection_vector;
					}

					delta.normalize();

					/*
					if (force_ > 0) delta = forceOrientation * osg::Y_AXIS;
					else delta = forceOrientation * -osg::Y_AXIS;
					*/

					// We now apply the force, scaling by amount of time,
					// and distance/angular decay factors:
					delta *= abs(force_) * dt * distanceScalar * angularScalar;

					// if the force is an attraction (negative) and this update
					// will bypass our location, then scale delta so the target
					// doesn't pass by:
					if (( connection_vector.length() - delta.length() < 0 ) && (force_ < 0))
					{
						delta *=  connection_vector.length() / delta.length();
					}

					if (delta.length() > EPSILON) // != osg::Vec3(0,0,0))
					{
						if (mode_ == AttractorNode::EXTRINSIC)
						{
							(*t)->translate( delta.x(), delta.y(), delta.z() );
						}
						else
						{
							osg::Vec3 v = (*t)->getVelocity();
							(*t)->setVelocity( v.x()+delta.x(), v.y()+delta.y(), v.z()+delta.z() );
						}

					}

				}
        	}
            lastTick_ = tick;
        }
    }

}

// *****************************************************************************
// setter methods

void AttractorNode::setDistanceDecay (float decay)
{
	if (distanceDecay_ != decay)
	{
		distanceDecay_ = decay;
		BROADCAST(this, "sf", "setDistanceDecay", getDistanceDecay());
	}
}
void AttractorNode::setAngularDecay (float decay)
{
	if (angularDecay_ != decay)
	{
		angularDecay_ = decay;
		BROADCAST(this, "sf", "setAngularDecay", getAngularDecay());
	}
}
void AttractorNode::setForce (float force)
{
	if (force_ != force)
	{
		force_ = force;
		BROADCAST(this, "sf", "setForce", getForce());
	}
}
void AttractorNode::setAttractorMode (attractorMode m)
{
	if (mode_ != m)
	{
		mode_ = m;
		BROADCAST(this, "si", "setAttractorMode", getAttractorMode());
	}
}

// *****************************************************************************
void AttractorNode::addTarget (const char *targetID)
{
	// target must inherit from GroupNode:
	osg::observer_ptr<GroupNode> n = dynamic_cast<GroupNode*>(sceneManager->getNode(targetID));

	if (!n.valid())
	{
		std::cout << "WARNING: AttractorNode '" << this->id->s_name << "' tried to addTarget '" << targetID << "', but node could be found, or is immovable" << std::endl;
		return;
	}

	// check if node is already in the list:
	targetVector::iterator t;
    for (t = targets_.begin(); t != targets_.end(); t++)
    {
    	if (n.get()==(*t).get())
    	{
    		//std::cout << "WARNING: AttractorNode '" << this->id->s_name << "' tried to addTarget '" << targetID << "', but that node is already in the list" << std::endl;
    		return;
    	}
    }

    // add it to the list:
    targets_.push_back(n);

    BROADCAST(this, "ss", "addTarget", targetID);
}

void AttractorNode::removeTarget (const char *targetID)
{
	// get ReferencedNode for the ID:
	osg::ref_ptr<GroupNode> n =  dynamic_cast<GroupNode*>(sceneManager->getNode(targetID));

	if (!n.valid())
	{
		std::cout << "WARNING: AttractorNode '" << this->id->s_name << "' tried to removeTarget '" << targetID << "', but no node by that name could be found" << std::endl;
		return;
	}

	// find it in the list, and remove it:
	targetVector::iterator t;
    for (t = targets_.begin(); t != targets_.end(); t++)
    {
    	if (n.get()==(*t).get())
    	{
    		targets_.erase(t);
    		BROADCAST(this, "ss", "removeTarget", targetID);
    		return;
    	}
    }
    std::cout << "WARNING: AttractorNode '" << this->id->s_name << "' tried to removeTarget '" << targetID << "', but that node was not in the target list" << std::endl;
}


// *****************************************************************************


std::vector<lo_message> AttractorNode::getState () const
{
    // inherit state from base class
    std::vector<lo_message> ret = GroupNode::getState();

    lo_message msg;
    osg::Vec3 v;

    targetVector::const_iterator t;
    for (t = targets_.begin(); t != targets_.end(); t++)
    {
    	if ((*t).valid())
		{
    		msg = lo_message_new();
    		lo_message_add(msg, "ss", "addTarget", (*t)->id->s_name);
    		ret.push_back(msg);
		}
    }

    msg = lo_message_new();
    lo_message_add(msg, "sf", "setDistanceDecay", getDistanceDecay());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "sf", "setAngularDecay", getAngularDecay());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "si", "setAttractorMode", getAttractorMode());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "sf", "setForce", getForce());
    ret.push_back(msg);

    return ret;
}

