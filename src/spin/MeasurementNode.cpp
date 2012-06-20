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

#include "MeasurementNode.h"
#include "SceneManager.h"
#include "spinApp.h"
#include "spinBaseContext.h"
#include "osgUtil.h"

using namespace std;

namespace spin
{

// *****************************************************************************
// constructor:
MeasurementNode::MeasurementNode (SceneManager *sceneManager, char *initID) : ReferencedNode(sceneManager, initID)
{
    this->setName(string(id->s_name) + ".MeasurementNode");
    nodeType = "MeasurementNode";

    targetName_ = gensym("NULL");
    reportingLevel_ = MeasurementNode::REPORT_BASIC;

    thisMatrix_.makeIdentity();
    targetMatrix_.makeIdentity();

    this->setNodeMask(STATSDATA_NODE_MASK); // nodemask info in spinUtil.h
}

// *****************************************************************************
// destructor
MeasurementNode::~MeasurementNode()
{

}

void MeasurementNode::callbackUpdate(osg::NodeVisitor* nv)
{
    ReferencedNode::callbackUpdate(nv);

    osg::ref_ptr<ReferencedNode> targetNode = dynamic_cast<ReferencedNode*>(targetName_->s_thing);
    if (!targetNode.valid()) return;

    osg::Matrixd mthis = osg::computeLocalToWorld(this->currentNodePath);
    osg::Matrixd mtarget = osg::computeLocalToWorld(targetNode->currentNodePath);

    // check if there is a change in the matrices:
    if ((thisMatrix_==mthis) && (targetMatrix_==mtarget))
    {
        return;
    } else
    {
        thisMatrix_ = mthis;
        targetMatrix_ = mtarget;
    }
    if ((int)reportingLevel_ > 0) sendMeasurements();
}

void MeasurementNode::sendMeasurements()
{
    std::vector<lo_message> msgs;
    lo_message msg;

    osg::Vec3 srcTrans, snkTrans, srcScale, snkScale;
    osg::Quat srcQuat, snkQuat, srcScaleQuat, snkScaleQuat;

    thisMatrix_.decompose(srcTrans, srcQuat, srcScale, srcScaleQuat);
    targetMatrix_.decompose(snkTrans, snkQuat, snkScale, snkScaleQuat);

    osg::Vec3 connection_vector = snkTrans - srcTrans;


    // let's also compute the orientations projected on the (local) horizontal
    // and vertical planes (ie, azimuth and elevation respectively)

    osg::Vec3 src_dir   = srcQuat * osg::Y_AXIS;
    osg::Vec3 src_right = srcQuat * osg::X_AXIS;
    osg::Vec3 src_up    = srcQuat * osg::Z_AXIS;

    osg::Vec3 snk_dir   = snkQuat * osg::Y_AXIS;
    osg::Vec3 snk_right = snkQuat * osg::X_AXIS;
    osg::Vec3 snk_up    = snkQuat * osg::Z_AXIS;

    // NOTE: all output angles are in RADIANS!

    if (reportingLevel_ > 0)
    {
        // direction: angle of connection_vector (projected on XY plane):
        float direction = AngleBetweenVectors(connection_vector, osg::Y_AXIS, 3);

        msg = lo_message_new();
        lo_message_add( msg, "sf", "distance", connection_vector.length() );
        msgs.push_back(msg);

        msg = lo_message_new();
        lo_message_add( msg, "sf", "direction", direction );
        msgs.push_back(msg);

        double incidence = AngleBetweenVectors(src_dir, connection_vector);

        msg = lo_message_new();
        lo_message_add( msg, "sf", "incidence", incidence );
        msgs.push_back(msg);
    }

    if (reportingLevel_ > 1)
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
        lo_message_add( msg, "sfff", "eulers", srcIncidenceElev, srcIncidenceRoll, srcIncidenceAzim);
        msgs.push_back(msg);

        osg::Quat rot;
        rot.makeRotate(connection_vector, src_dir);

        msg = lo_message_new();
        lo_message_add(msg, "sffff", "quaternion", rot.x(), rot.y(), rot.z(), rot.w());
        msgs.push_back(msg);

        osg::Vec3 rotEulers = QuatToEuler(rot);

        msg = lo_message_new();
        lo_message_add( msg, "sfff", "test", rotEulers.x(), rotEulers.y(), rotEulers.z());
        msgs.push_back(msg);
    }
    spinApp::Instance().NodeBundle(this->id, msgs);
}

// *****************************************************************************
void MeasurementNode::setTarget (const char *targetID)
{
    if (targetName_ != gensym(targetID))
    {
        targetName_ = gensym(targetID);
        BROADCAST(this, "ss", "setTarget", getTarget());
    }
}

void MeasurementNode::setReportingLevel (reportMode level)
{
    this->reportingLevel_ = level;
    BROADCAST(this, "si", "setReportingLevel", getReportingLevel());
}

// *****************************************************************************

std::vector<lo_message> MeasurementNode::getState () const
{
    // inherit state from base class
    std::vector<lo_message> ret = ReferencedNode::getState();

    lo_message msg;
    msg = lo_message_new();
    lo_message_add(msg, "ss", "setTarget", this->getTarget());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "si", "setReportingLevel", this->getReportingLevel() );
    ret.push_back(msg);

    return ret;
}

} // end of namespace spin

