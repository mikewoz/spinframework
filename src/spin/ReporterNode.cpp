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

#include "ReporterNode.h"
#include "SceneManager.h"
#include "spinApp.h"
#include "spinBaseContext.h"
#include "osgUtil.h"

#include <osgUtil/LineSegmentIntersector>
#include <osgUtil/IntersectionVisitor>

using namespace std;


// *****************************************************************************
// constructor:
ReporterNode::ReporterNode (SceneManager *sceneManager, char *initID) : ReferencedNode(sceneManager, initID)
{
    this->setName(string(id->s_name) + ".ReporterNode");
    nodeType = "ReporterNode";

	maxRate_ = 20.0;

    reporting_["DISTANCE"] = false;
    reporting_["INCIDENCE"] = false;
    reporting_["ANGLES"] = false;
    reporting_["CONTAINMENT"] = false;
    reporting_["OCCLUSION"] = false;

    matrix_.makeIdentity();

    this->setNodeMask(STATSDATA_NODE_MASK); // nodemask info in spinUtil.h

    lastTick = osg::Timer::instance()->tick();
}

// *****************************************************************************
// destructor
ReporterNode::~ReporterNode()
{
	targets_.clear();
	reporting_.clear();
}

void ReporterNode::callbackUpdate()
{
    ReferencedNode::callbackUpdate();

    // only report from the server
    if (spinApp::Instance().getContext()->isServer()) return;

    // limit rate of updates:
    osg::Timer_t tick = osg::Timer::instance()->tick();
    float dt = osg::Timer::instance()->delta_s(lastTick,tick);
    if (dt > 1/maxRate_) // only update when dt is at least 0.05s (ie 20hz):
    {
        bool needReport = false;

		osg::Matrixd mthis = osg::computeLocalToWorld(this->currentNodePath);
		if (matrix_!=mthis)
		{
			needReport = true;
			matrix_ = mthis;
		}

		std::vector<reporterTarget>::iterator t;
		for (t = targets_.begin(); t != targets_.end();)
		{
			if ((*t).node.valid())
			{
				osg::Matrixd mtarget = osg::computeLocalToWorld((*t).node->currentNodePath);

				// if there is a change in this node's matrix or the target, we need
				// to send an updated report:
				if (needReport || ((*t).matrix!=mtarget))
				{
					(*t).matrix = mtarget;
					sendReports(&(*t));
				}
				t++;
			}

			else
			{
				// the node has been deleted, so we need to remove this target from
				// the list
				targets_.erase(t); // t++ is implicit
			}
		}

		lastTick = tick;
	}
}

void ReporterNode::sendReports(reporterTarget *target)
{
	if (spinApp::Instance().getContext()->isServer()) return;

	// check that at least one report is required:
	bool allDisabled = true;
    for (ReportingType::iterator i=reporting_.begin(); i!=reporting_.end(); i++)
    {
	    if (i->second) allDisabled = false;
    }
	if (allDisabled) return;


    std::vector<lo_message> msgs;
    lo_message msg;

    osg::Vec3 srcTrans, snkTrans, srcScale, snkScale;
    osg::Quat srcQuat, snkQuat, srcScaleQuat, snkScaleQuat;

    matrix_.decompose(srcTrans, srcQuat, srcScale, srcScaleQuat);
    target->matrix.decompose(snkTrans, snkQuat, snkScale, snkScaleQuat);

    osg::Vec3 connection_vector = snkTrans - srcTrans;

    // get the direction vectors from the quaternion by projection:
    osg::Vec3 src_right = srcQuat * osg::X_AXIS;
    osg::Vec3 src_dir   = srcQuat * osg::Y_AXIS;
    osg::Vec3 src_up    = srcQuat * osg::Z_AXIS;

    osg::Vec3 snk_right = snkQuat * osg::X_AXIS;
    osg::Vec3 snk_dir   = snkQuat * osg::Y_AXIS;
    osg::Vec3 snk_up    = snkQuat * osg::Z_AXIS;

    // NOTE: all output angles are in RADIANS!




    if (reporting_["DISTANCE"])
    {
    	msg = lo_message_new();
    	lo_message_add( msg, "ssf", "distance", target->node->id->s_name, connection_vector.length() );
    	msgs.push_back(msg);
    }

    if (reporting_["INCIDENCE"])
    {
    	// incidence: angle difference between current orientation to that which
    	// would point at the target
        double incidence = AngleBetweenVectors(src_dir, connection_vector);

        msg = lo_message_new();
        lo_message_add( msg, "ssf", "incidence", target->node->id->s_name, incidence );
        msgs.push_back(msg);

        // direction: angle of connection_vector (projected on XY plane):
        float direction = AngleBetweenVectors(connection_vector, osg::Y_AXIS, 3);

        msg = lo_message_new();
        lo_message_add( msg, "ssf", "direction", target->node->id->s_name, direction );
        msgs.push_back(msg);

    }

    if (reporting_["ANGLES"])
    {
        // Elevation: source incidence projected on XZ plane (Y is ignored)
        //float srcIncidenceElev = AngleBetweenVectors(src_up, connection_vector, 2);
        float srcIncidenceElev = AngleBetweenVectors(src_dir, connection_vector, 2);

        // Roll: source incidence projected on YZ plane (X is ignored)
        //float srcIncidenceRoll = AngleBetweenVectors(src_right, connection_vector, 1);
        float srcIncidenceRoll = AngleBetweenVectors(src_dir, connection_vector, 1);

        // Azimuth: source incidence projected on XY plane (Z is ignored)
        float srcIncidenceAzim = AngleBetweenVectors(src_dir, connection_vector, 3);

        msg = lo_message_new();
        lo_message_add( msg, "ssfff", "eulers", target->node->id->s_name, srcIncidenceElev, srcIncidenceRoll, srcIncidenceAzim);
        msgs.push_back(msg);

        osg::Quat rot;
        rot.makeRotate(connection_vector, src_dir);

        msg = lo_message_new();
        lo_message_add(msg, "ssffff", "quaternion", target->node->id->s_name, rot.x(), rot.y(), rot.z(), rot.w());
        msgs.push_back(msg);

        /*
        osg::Vec3 rotEulers = QuatToEuler(rot);

        msg = lo_message_new();
        lo_message_add( msg, "ssfff", "test", target->node->id->s_name, rotEulers.x(), rotEulers.y(), rotEulers.z());
        msgs.push_back(msg);
        */
    }

    if (reporting_["CONTAINMENT"])
    {
        osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector =
            new osgUtil::LineSegmentIntersector(srcTrans, snkTrans);

        osgUtil::IntersectionVisitor iv(intersector.get());

        target->node->accept(iv);

		// TODO: report distance (and normal?) to surface
        /*
        if (intersector->containsIntersections())
        {
        	osg::Vec3 intersector->getFirstIntersection().getWorldIntersectPoint();
        }
        */

        if (target->contained != (!intersector->containsIntersections()))
        {
        	target->contained = !intersector->containsIntersections();
			msg = lo_message_new();
			lo_message_add(msg, "ssi", "containedBy", target->node->id->s_name, (int)target->contained);
			msgs.push_back(msg);
        }
    }

    if (reporting_["OCCLUSION"])
    {
    	// TODO
    }

    if (msgs.size())
    	spinApp::Instance().NodeBundle(this->id, msgs);
}

// *****************************************************************************
void ReporterNode::addTarget (const char *targetID)
{
	// get ReferencedNode for theat ID:
	osg::ref_ptr<ReferencedNode> n = sceneManager->getNode(targetID);

	if (!n.valid())
	{
		std::cout << "WARNING: reporterNode '" << this->id->s_name << "' tried to addTarget '" << targetID << "', but no node by that name could be found" << std::endl;
		return;
	}

	// check if node is already in the list:
    std::vector<reporterTarget>::iterator t;
    for (t = targets_.begin(); t != targets_.end(); t++)
    {
    	if (n.get()==(*t).node.get())
    	{
    		//std::cout << "WARNING: reporterNode '" << this->id->s_name << "' tried to addTarget '" << targetID << "', but that node is already in the list" << std::endl;
    		return;
    	}
    }

    // add it to the list:
    reporterTarget newTarget;
    newTarget.node = n.get();
    newTarget.matrix = osg::computeLocalToWorld(n->currentNodePath);
    newTarget.contained = false;
    targets_.push_back(newTarget);


    BROADCAST(this, "ss", "addTarget", targetID);

    // force initial report:
    this->sendReports(&newTarget);
}

void ReporterNode::removeTarget (const char *targetID)
{
	// get ReferencedNode for the ID:
	osg::ref_ptr<ReferencedNode> n = sceneManager->getNode(targetID);

	if (!n.valid())
	{
		std::cout << "WARNING: reporterNode '" << this->id->s_name << "' tried to removeTarget '" << targetID << "', but no node by that name could be found" << std::endl;
		return;
	}

	// find it in the list, and remove it:
    std::vector<reporterTarget>::iterator t;
    for (t = targets_.begin(); t != targets_.end(); t++)
    {
    	if (n.get()==(*t).node.get())
    	{
    		targets_.erase(t);
    		BROADCAST(this, "ss", "removeTarget", targetID);
    		break;
    	}
    }
}

void ReporterNode::setReporting (const char *type, bool enabled)
{
	// make sure type exists:
	ReportingType::iterator typeKey = reporting_.find(type);
	if (typeKey!=reporting_.end())
	{
		reporting_[type] = enabled;

		BROADCAST(this, "ssi", "setReporting", type, getReporting(type));

		// if we've just turned on a report mode, we need to force one
		// computation for all  targets:
		//if (enabled) callbackUpdate();

		if (enabled)
		{
			std::vector<reporterTarget>::iterator t;
			for (t = targets_.begin(); t != targets_.end(); t++)
			{
				if ((*t).node.valid())
				{
					(*t).matrix = osg::computeLocalToWorld((*t).node->currentNodePath);
					this->sendReports(&(*t));
				}
			}
		}

	}
}

void ReporterNode::setMaxRate(float hz)
{
	maxRate_ = hz;
	BROADCAST(this, "sf", "setMaxRate", maxRate_);
}


// *****************************************************************************

std::vector<lo_message> ReporterNode::getState () const
{
    // inherit state from base class
    std::vector<lo_message> ret = ReferencedNode::getState();
    lo_message msg;

    ReportingType::const_iterator i;
    for (i=reporting_.begin(); i!=reporting_.end(); i++)
    {
        msg = lo_message_new();
        lo_message_add(msg, "ssi", "setReporting", i->first.c_str(), (int)i->second );
        ret.push_back(msg);
    }

    std::vector<reporterTarget>::const_iterator t;
    for (t = targets_.begin(); t != targets_.end(); t++)
    {
    	if ((*t).node.valid())
		{
    		msg = lo_message_new();
    		lo_message_add(msg, "ss", "addTarget", (*t).node->id->s_name);
    		ret.push_back(msg);
		}
    }

	msg = lo_message_new();
	lo_message_add(msg, "sf", "setMaxRate", getMaxRate());
    ret.push_back(msg);


    return ret;
}
	
int ReporterNode::getReporting(const char *type) const 
{ 
    // don't do return (int)reporting_[type] because if report_[type] doesn't 
    // exist, it will be created with a default value
    std::map<std::string, bool>::const_iterator iter = reporting_.find(type);
    if (iter != reporting_.end())
        return iter->second;
    else
        return 0; // default value for bool == false, which as an int will be 0
}



