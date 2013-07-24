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

#include <osgManipulator/CommandManager>
#include <osgManipulator/TabBoxDragger>
#include <osgManipulator/TabPlaneDragger>
#include <osgManipulator/TabPlaneTrackballDragger>
#include <osgManipulator/TrackballDragger>
#include <osgManipulator/Translate1DDragger>
#include <osgManipulator/Translate2DDragger>
#include <osgManipulator/TranslateAxisDragger>
#include <osgUtil/IntersectionVisitor>

#include "raynode.h"
#include "groupnode.h"
#include "scenemanager.h"
#include "spinapp.h"
#include "spinbasecontext.h"
#include "osgutil.h"
#include "spinutil.h"
#include "pointernode.h"

extern pthread_mutex_t sceneMutex;

namespace spin
{

// *****************************************************************************
// constructor:
PointerNode::PointerNode (SceneManager *sceneManager, const char* initID) : RayNode(sceneManager, initID)
{
    this->setName(this->getID() + ".PointerNode");
    this->setNodeType("PointerNode");

    lastManipulatorType_ = "TabBoxDragger";
    //draggerType = "TabPlaneDragger";
    //draggerType = "TrackballDragger";
    //draggerType = "RotateCylinderDragger";
    //draggerType = "Grabber";

    // create a command manager
    //cmdMgr = new osgManipulator::CommandManager;

    //selection = new osgManipulator::Selection;
    //selection->setName("PointerNode.selection");


    ea = new osgGA::GUIEventAdapter();
    ea->setWindowRectangle(1, 1, 1, 1);

    doManipulation = false;

    slideIncrement_ = 0.0;
    rollIncrement_ = 0.0;

    grabbedNode = NULL;
    targetNode = NULL;

    lastManipulated = gensym("NULL");

    intersectList.clear();


    // We use a PointerInfo object to interface with OSG's GUIEventAdapter class
    // Usually PointerInfo is used to construct a linesegment based on mouse x,y
    // projection into 3D space. PointerInfo's method projectWindowXYIntoObject
    // is used to create the line segment, which needs a camera, but we don't
    // need that (since we already have a 3D linesegment).
    _pointer.setCamera(NULL);


    this->setNodeMask(DEBUGVIEW_NODE_MASK); // nodemask info in spinUtil.h

}

// *****************************************************************************
// destructor
PointerNode::~PointerNode()
{

}

void PointerNode::callbackUpdate(osg::NodeVisitor* nv)
{
    RayNode::callbackUpdate(nv);

    this->computeBound();


    // TODO: It's too much work to perform an intersection traversal on all
    // objects in the scenegraph. We should add a way to limit this to only
    // nodes with a certain nodemask so that the user can optimize and choose
    // which parts of the scene should be included / excluded.

    //osg::Timer_t startTick = osg::Timer::instance()->tick();



    // get line segment start and end points:

    osg::Matrix myMatrix = osg::computeLocalToWorld(this->currentNodePath_);

    osg::Vec3 t;
    osg::Quat q;
    osg::Vec3 s;
    osg::Quat so;
    myMatrix.decompose(t, q, s, so); // <- to heavy?

    osg::Vec3 start = t;
    osg::Vec3 end = t + ( q * osg::Vec3(0.0,this->getLength(),0.0) );




    // create an intersector and create an intersectorVisitor.
    //osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector = new osgUtil::LineSegmentIntersector(start, end);
    intersector = new osgUtil::LineSegmentIntersector(start, end);
    osgUtil::IntersectionVisitor intersectVisitor( intersector.get() );

    // make sure to provide a nodemask so that only geometric nodes are checked
    // for intersections:
    intersectVisitor.setTraversalMask(INTERACTIVE_NODE_MASK);

    // start the visitor:
    sceneManager_->worldNode->accept(intersectVisitor);

    //osg::Timer_t endTick = osg::Timer::instance()->tick();
    //std::cout<<"Intersection completed in "<<osg::Timer::instance()->delta_s(startTick,endTick)<<std::endl;


    if (spinApp::Instance().getContext()->isServer())
    {
        reportIntersections();

        if (grabbedNode.valid())
        {
            applyGrab(myMatrix);
        }
    }

    //if (!spinApp::Instance().getContext()->isServer())
    else
    {
        applyManipulation(myMatrix, start, end);
    }

    this->previousMatrix = myMatrix;
}

void PointerNode::applyGrab(osg::Matrix mat)
{
    if (this->previousMatrix != mat)
    {

        osg::Matrix newMatrix;
        osg::Matrix diff = osg::Matrix::inverse(origPointerMatrix) * mat;

        /*
        std::cout << "Grabbed nodepath:" << std::endl;
        for (osg::NodePath::iterator itr = grabbedNode->currentNodePath_.begin(); itr != grabbedNode->currentNodePath_.end(); ++itr)
        {
            std::cout << "   -> " << (*itr)->getName() << std::endl;
        }

        std::cout << "diff      pos: " << stringify(diff.getTrans()) << std::endl;
        std::cout << "diff      rot: " << stringify(diff.getRotate()) << std::endl;
        std::cout << "diff      eul: " << stringify(QuatToEuler(diff.getRotate())) << std::endl;
        */



        osg::Matrix grabbedLocalToWorld;
        ReferencedNode *parentNode = grabbedNode->getParentNode(0);
        if (parentNode)
        {
            grabbedLocalToWorld = osg::computeWorldToLocal(parentNode->currentNodePath_);
        } else
        {
            grabbedLocalToWorld = osg::Matrix::identity();
            //grabbedLocalToWorld = osg::computeWorldToLocal(grabbedNode->currentNodePath_);
        }


        /*
        // NOTE: this is the test that work to fix parent offset
        osg::NodePath newPath = grabbedNode->currentNodePath_;
        newPath.pop_back();
        newPath.pop_back();
        std::cout << "newpath:" << std::endl;
        for (osg::NodePath::iterator itr = newPath.begin(); itr != newPath.end(); ++itr)
        {
            std::cout << "   -> " << (*itr)->getName() << std::endl;
        }

        osg::Matrix grabbedWorldToLocal2 = osg::Matrix::inverse(osg::computeLocalToWorld(newPath));
        */

        osg::Matrix slideMatrix = osg::Matrix::translate(osg::Vec3(0.0,slideIncrement_,0.0));

        newMatrix =  grabbedLocalToWorld * origGrabbedMatrix * slideMatrix * diff;
        grabbedNode->setTranslation(newMatrix.getTrans().x(), newMatrix.getTrans().y(), newMatrix.getTrans().z());

        if (grabMode_ == RELATIVE)
        {
            newMatrix = grabbedLocalToWorld * mat;
            osg::Quat rot = newMatrix.getRotate();
            grabbedNode->setOrientationQuat(rot.x(), rot.y(), rot.z(), rot.w());
        }

    }
}

void PointerNode::applyManipulation(osg::Matrix mat, osg::Vec3 start, osg::Vec3 end)
{

    // If the user is holding down the manipulate button, and the dragger has
    // not yet been created, then we need to create a dragger
    if ((doManipulation) && (!dragger_.valid()))
    {
        // Reset the PointerInfo object (clear old intersections)
        _pointer.reset();

        if ( !intersector->containsIntersections() )
        {
            //std::cout << "no intersections" <<std::endl;
            return;
        }


        // Transfer the intersections to the PointerInfo object:
        osgUtil::LineSegmentIntersector::Intersections& intersections = intersector->getIntersections();
        osgUtil::LineSegmentIntersector::Intersections::iterator hitr;
        for (hitr = intersections.begin(); hitr != intersections.end(); ++hitr)
        {

            _pointer.addIntersection(hitr->nodePath, hitr->getLocalIntersectPoint());
        }


        //_pointer._hitIter = _pointer._hitList.begin();

        bool didDragger = false;

        for (osg::NodePath::iterator itr = _pointer._hitList.front().first.begin();
             itr != _pointer._hitList.front().first.end();
             ++itr)
        {

            osgManipulator::Dragger* ptrDragger = dynamic_cast<osgManipulator::Dragger*>(*itr);
            if (ptrDragger)
            {
                // Update the PointerInfo by setting the start and end points
                // based on our PointerNode ray:
                _pointer.setNearFarPoints(start,end);

                ea->setEventType(osgGA::GUIEventAdapter::PUSH);
                ptrDragger->handle(_pointer, *ea.get(), aa);
                ptrDragger->setDraggerActive(true);
                dragger_ = ptrDragger;

                didDragger = true;
                //break;
            }
        }

        // If the ...

    }

    // if the dragger is already valid, then just do the DRAG
    //else if (doManipulation && dragger.valid())
    else if (doManipulation && dragger_.valid() && (this->previousMatrix != mat))
    {
        // Update the PointerInfo by setting the start and end points based on our
        // PointerNode ray:
        _pointer.setNearFarPoints(start,end);
        _pointer._hitIter = _pointer._hitList.begin();

        //ea->setHandled(false);
        ea->setEventType(osgGA::GUIEventAdapter::DRAG);
        dragger_->handle(_pointer, *ea.get(), aa);

    }

    // if the dragger is valid and the manipulator flag has been set to off,
    // then we release the dragger:
    else if (!doManipulation && (dragger_.valid()))
    {
        //  set event to RELEASE and invoke handle()
        _pointer.setNearFarPoints(start,end);
        _pointer._hitIter = _pointer._hitList.begin();

        ea->setEventType(osgGA::GUIEventAdapter::RELEASE);
        dragger_->handle(_pointer, *ea.get(), aa);
        dragger_->setDraggerActive(false);
        _pointer.reset();

        dragger_ = NULL;
    }

}



void PointerNode::reportIntersections()
{

    // Store and report our intersections:

    std::vector<ReferencedNode*> newIntersectList;
    std::vector<osgUtil::LineSegmentIntersector::Intersection> newIntersectData;
    std::vector<osg::Vec3> newItersectListOffsets;
    std::vector<ReferencedNode*>::iterator iter;

    if ( intersector->containsIntersections() )
    {
        osg::ref_ptr<ReferencedNode> testNode;

        osgUtil::LineSegmentIntersector::Intersections& intersections = intersector->getIntersections();
        osgUtil::LineSegmentIntersector::Intersections::iterator itr;

        for (itr = intersections.begin(); itr != intersections.end(); ++itr)
        {
            const osgUtil::LineSegmentIntersector::Intersection& intersection = *itr;

            for (int i=intersection.nodePath.size()-1; i>=0; i--)
            {

                osgManipulator::Dragger* dragger = dynamic_cast<osgManipulator::Dragger*>(intersection.nodePath[i]);
                if (dragger)
                {
                    //std::cout << " GOT INTERSECTION with dragger:" << std::endl;
                    //break;
                }

                testNode = dynamic_cast<ReferencedNode*>(intersection.nodePath[i]);
                if (testNode.valid()) // && (testNode->nodeType != "RayNode"))
                {
                    // only add the hit if not already in the list (note that
                    // one node might have several intersections, for each face
                    // that instersects with the line segment)
                    iter = std::find( newIntersectList.begin(), newIntersectList.end(), testNode );
                    if ( iter == newIntersectList.end() )
                    {
                        newIntersectList.push_back(testNode);
                        newIntersectData.push_back(intersection);
                        newItersectListOffsets.push_back(intersection.getLocalIntersectPoint());
                    }
                    break;
                }
                //std::cout << " > " << intersection.nodePath[i]->getName();
            }

            /*
            std::cout << "Got intersection: " << std::endl;
            osg::Vec3 dbgVect;
            std::cout<<"  ratio "<<intersection.ratio<<std::endl;
            dbgVect = intersection.getLocalIntersectPoint();
            std::cout<<"  localPoint  ("<<stringify(dbgVect)<<")"<<std::endl;
            dbgVect = intersection.getLocalIntersectNormal();
            std::cout<<"  localNormal ("<<stringify(dbgVect)<<")"<<std::endl;
            dbgVect = intersection.getWorldIntersectPoint();
            std::cout<<"  worldPoint  ("<<stringify(dbgVect)<<")"<<std::endl;
            dbgVect = intersection.getWorldIntersectNormal();
            std::cout<<"  worldNormal ("<<stringify(dbgVect)<<")"<<std::endl;
            */
        }

        /*
        std::cout << " OLD intersections:";
        for (i=0; i<intersectList.size(); i++) std::cout << " " << intersectList[i]->s_name;
        std::cout << std::endl;

        std::cout << " NEW intersections:";
        for (i=0; i<newIntersectList.size(); i++) std::cout << " " << newIntersectList[i]->s_name;
        std::cout << std::endl;
        */

    }

    // Check to see if the intersectList has changed from the previous one:
    bool intersectChange = false;
    if (intersectList.size()==newIntersectList.size())
    {
        if (!std::equal(intersectList.begin(), intersectList.end(), newIntersectList.begin())) intersectChange = true;
    } else intersectChange = true;

    // if the intersectList has changed, then we replace our old one and
    // broadcast the new list:
    if (intersectChange)
    {

        if (0) // debug print
        {
            osgUtil::LineSegmentIntersector::Intersections& intersections = intersector->getIntersections();
            osgUtil::LineSegmentIntersector::Intersections::iterator itr;

            std::cout << std::endl;
            std::cout << "GOT " << intersections.size() << " INTERSECTIONS:" << std::endl;

            int count=1;
            for (itr = intersections.begin(); itr != intersections.end(); ++itr)
            {
                const osgUtil::LineSegmentIntersector::Intersection& intersection = *itr;

                std::cout << count++ << ")" << std::endl;
                //std::cout << "  nodepath:";
                for (int i=intersection.nodePath.size()-1; i>=0; i--)
                {
                    if (intersection.nodePath[i]->getName().empty()) std::cout << " ?";
                    else std::cout << " " << intersection.nodePath[i]->getName();

                    osg::Node* n = dynamic_cast<osg::Node*>(intersection.nodePath[i]);

                    if (n) std::cout << ",  bound: " << stringify(n->getBound().center()) << ", radius=" << n->getBound().radius() << std::endl;

                } std::cout << std::endl;
            }
        }



        // TODO: we don't need all three of these lists. The intersectData
        // list has everything we need (ie, contains the other two)

        intersectList.clear();
        intersectList.resize(newIntersectList.size());
        std::copy(newIntersectList.begin(), newIntersectList.end(), intersectList.begin());

        intersectListOffsets.clear();
        intersectListOffsets.resize(newItersectListOffsets.size());
        std::copy(newItersectListOffsets.begin(), newItersectListOffsets.end(), intersectListOffsets.begin());

        intersectData.clear();
        intersectData.resize(newIntersectData.size());
        std::copy(newIntersectData.begin(), newIntersectData.end(), intersectData.begin());

        lo_message msg = lo_message_new();
        lo_message_add_string(msg, "intersectsWith");
        if (intersectList.size())
        {
            for (int i=0; i<intersectList.size(); i++)
            {
                ReferencedNode *n = getNodeFromIntersections(i);
                if (n)
                    lo_message_add_string(msg, (char*)n->getID().c_str());
            }
        }
        else
            lo_message_add_string(msg, "NULL");

        // debug:
        if (0)
        {
            std::cout << "PointerNode has new intersections:";
            if (intersectList.size())
            {
                for (int i=0; i<intersectList.size(); i++)
                    std::cout << " " << intersectList[i]->getID();
            } else std::cout << " NULL";
            std::cout << std::endl;
        }
        // end debug

        //sceneManager_->sendNodeMessage(this->id, msg);
        if (spinApp::Instance().getContext()->isServer())
        {
            //NODE_LO_MSG(sceneManager, this, msg);
            spinApp::Instance().BroadcastMessage(this->getOSCPath(), msg);
        }
    }

}

// *****************************************************************************

GroupNode *PointerNode::getNodeFromIntersections(int index)
{
    // return first GroupNode encountered with interaction mode greater than passthru

    if ((index>=0) && (index < intersectList.size()))
    {
        ReferencedNode *t = intersectList[index];
        while (t)
        {
            GroupNode *g = dynamic_cast<GroupNode*>(t);
            if (g && (g->getInteractionMode()>(int)GroupNode::PASSTHRU))
            {
                return g;
            }
            t = dynamic_cast<ReferencedNode*>(t->getParentNode(0));
        }
    }

    return NULL;
}

osgManipulator::Dragger* PointerNode::getDraggerFromIntersections()
{
    osgUtil::LineSegmentIntersector::Intersections& intersections = intersector->getIntersections();
    osgUtil::LineSegmentIntersector::Intersections::iterator itr;

    for (itr = intersections.begin(); itr != intersections.end(); ++itr)
    {
        const osgUtil::LineSegmentIntersector::Intersection& intersection = *itr;

        for (int i=intersection.nodePath.size()-1; i>=0; i--)
        {

            osgManipulator::Dragger* dragger = dynamic_cast<osgManipulator::Dragger*>(intersection.nodePath[i]);
            if (dragger)
            {
                return dragger;
            }
        }
    }
    return NULL;
}


/*
void PointerNode::computeRT(t_symbol *src, t_symbol *dst, osg::Vec3 &R, osg::Vec3 &T)
{

    osg::Matrix srcMatrix, dstMatrix;

    if (src==gensym("world"))
        srcMatrix = osg::Matrix::identity();
    else
        srcMatrix = osg::computeLocalToWorld(src->s_thing->currentNodePath_);

    if (dst==gensym("world"))
        dstMatrix = osg::Matrix::identity();
    else
        dstMatrix = osg::computeLocalToWorld(dst->s_thing->currentNodePath_);

    // find change in translation and orientation from src to dst:


    osg::Quat q1 = srcMatrix.getRotate();
    //osg::Quat q2 = dstMatrix.getRotate();
    //Q = q2 - q1;


    osg::Vec3 r1 = Vec3inDegrees(QuatToEuler(srcMatrix.getRotate()));
    osg::Vec3 r2 = Vec3inDegrees(QuatToEuler(dstMatrix.getRotate()));
    R = r2 - r1;

    osg::Vec3 p1 = srcMatrix.getTrans();
    osg::Vec3 p2 = dstMatrix.getTrans();
    osg::Vec3 dp = p2 - p1;

    // position of dst if projected on src direction vector:
    T = p1 + (q1 * osg::Vec3(0,dp.length(),0)) + grabbedOffset;
}
*/

// *****************************************************************************

void PointerNode::lockToTarget(const char *nodeToLock)
{
    if (!spinApp::Instance().getContext()->isServer())
        return;

    GroupNode *node = dynamic_cast<GroupNode*>(sceneManager_->getNode(nodeToLock));
    if (node)
    {
        GroupNode *target = dynamic_cast<GroupNode*>(getNodeFromIntersections(0));
        if (target)
        {
            node->setOrientationMode(GroupNode::POINT_TO_TARGET_CENTROID);
            node->setOrientationTarget(target->getID().c_str());
        }
        else
        {
            // What do we do if there is no target?
            node->setOrientationTarget(lastManipulated->s_name);
        }
    }

}

void PointerNode::setManipulator(const char *manipulatorType)
{
    if (spinApp::Instance().getContext()->isServer())
    {
        GroupNode *lastNode = dynamic_cast<GroupNode*>(lastManipulated->s_thing);

        // see if there is an intersection with a GroupNode, and if so, tell
        // that node to enable the manipulator
        GroupNode *n = dynamic_cast<GroupNode*>(getNodeFromIntersections(0));
        if (n)
        {
            // if we're targetting a new node, make sure that the last
            // manipulated node's dragger gets turned off:
            if ((lastNode) && (n != lastNode))
                lastNode->setManipulator("NULL");

            n->setManipulator(manipulatorType);
            lastManipulated = n->getNodeSymbol();
        }

        // if there was no intersection, load the manipulator on the last object
        // that was manipulated
        else if (lastNode)
        {
            lastNode->setManipulator(manipulatorType);
        }

        lastManipulatorType_ = std::string(manipulatorType);
    }
    else
    {
        BROADCAST(this, "ss", "setManipulator", manipulatorType);
    }
}

void PointerNode::setGrabMode (GrabMode mode)
{
    this->grabMode_ = mode;
    BROADCAST(this, "si", "setGrabMode", getGrabMode());
}

void PointerNode::manipulate (int b)
{
    // Immediately when the user presses the manipulate button, we check to see
    // if we are pointing at a new node. ie, we make sure the user is NOT
    // pointing at a dragger anymore, and
    if (b && spinApp::Instance().getContext()->isServer())
    {
        if (1) //!getDraggerFromIntersections())
        {
            GroupNode *lastNode = dynamic_cast<GroupNode*>(lastManipulated->s_thing);
            GroupNode *newNode = dynamic_cast<GroupNode*>(getNodeFromIntersections(0));

            if (newNode && (newNode!=lastNode))
            {
                dragger_ = NULL;
                if (lastNode) lastNode->setManipulator("NULL");
                newNode->setManipulator(lastManipulatorType_.c_str());
                lastManipulated = newNode->getNodeSymbol();
            }
        }
    }

    // then we just set the 'doManipulation' flag, which will

    doManipulation = (bool) b;
    BROADCAST(this, "si", "manipulate", this->getManipulate());
}

void PointerNode::grab (int b)
{
    // return if this spinContext is a slave
    if (!spinApp::Instance().getContext()->isServer())
        return;

    osg::Matrix srcMatrix, dstMatrix;

    // start grab:
    if (b and intersectList.size())
    {

        // What do we do if a node is already grabbed? let go and grab again?
        // ... for now, let's do nothing.
        if (grabbedNode.valid()) return;

        grabbedNode = getNodeFromIntersections(0);
        osg::Vec3 localIntersectPt = intersectData[0].getLocalIntersectPoint();
        osg::Vec3 worldIntersectPt = intersectData[0].getWorldIntersectPoint();

        if (grabbedNode.valid())
        {
            const osgUtil::LineSegmentIntersector::Intersection& intersection = intersectData[0];

            /*

            std::cout << "Grabbed " << grabbedNode->getID() << std::endl;
            std::cout << "  localPt = " << stringify(localIntersectPt) << std::endl;
            std::cout << "  worldPt = " << stringify(worldIntersectPt) << std::endl;
            */
            origPointerMatrix = osg::computeLocalToWorld(this->currentNodePath_);
            origGrabbedMatrix = osg::computeLocalToWorld(grabbedNode->currentNodePath_);

            slideIncrement_ = 0.0;

            /*
           origPointerMatrix = osg::Matrix::rotate(mainTransform->getAttitude())
            * osg::Matrix::scale(mainTransform->getScale())
            * osg::Matrix::translate(mainTransform->getPosition());
            */

            osg::NodePath nodePathToRoot;
            osgManipulator::computeNodePathToRoot(*mainTransform_,nodePathToRoot);
            _localToWorld = osg::computeLocalToWorld(nodePathToRoot);
            _worldToLocal = osg::Matrix::inverse(_localToWorld);



        }
        else
        {
            std::cout << "Grab not valid" << std::endl;
            return;
        }


    }

    // end grab:
    else if (grabbedNode.valid())
    {

        grabbedNode = NULL;

    }

    BROADCAST(this, "si", "grab", this->getGrab());


}

void PointerNode::translateOnPointer (float f)
{
    if (grabbedNode.valid())
    {
        std::cout << "slideIncrement = " << f << std::endl;
        slideIncrement_ = f;
        return;
        osg::ref_ptr<GroupNode> n = dynamic_cast<GroupNode*>(grabbedNode.get());
        osg::Vec3 T = n->getTranslation() + osg::Vec3(0,f,0);

        // (TODO: use sceneManager_->invokeMethod)
        n->setTranslation(T.x(), T.y(), T.z());
    }
    else
        slideIncrement_ = 0.0;
}

void PointerNode::rotateOnPointer (float f)
{
    if (grabbedNode.valid())
    {

        // Note: don't use roll from pointer, since it is typically NOT updated. Usually only pitch/yaw ie, azim/elev are sent my a sensor to avoid gimbal issues.


        osg::ref_ptr<GroupNode> n = dynamic_cast<GroupNode*>(grabbedNode.get());

        osg::Vec3 t;
        osg::Quat q;
        osg::Vec3 s;
        osg::Quat so;
        origGrabbedMatrix.decompose(t, q, s, so);

        //std::cout << "Got twist of " << f << ", orig angles= " << stringify(Vec3inDegrees(QuatToEuler(q))) << std::endl;


        //osg::Quat newQuat = q * n->getOrientationQuat() ;
        osg::Quat newQuat = q * osg::Quat(osg::DegreesToRadians(f),osg::Y_AXIS);

        n->setOrientationQuat(newQuat.x(), newQuat.y(), newQuat.z(), newQuat.w());


        // old method
        /*
        osg::ref_ptr<GroupNode> n = dynamic_cast<GroupNode*>(grabbedNode.get());
        osg::Vec3 R = n->getOrientation() + osg::Vec3(0,f,0);
        n->setOrientation(R.x(), R.y(), R.z());
        */
    }
}



// *****************************************************************************

std::vector<lo_message> PointerNode::getState () const
{
    // inherit state from base class
    std::vector<lo_message> ret = RayNode::getState();

    lo_message msg;

    msg = lo_message_new();
    lo_message_add(msg, "si", "setGrabMode", getGrabMode());
    ret.push_back(msg);

/*
    msg = lo_message_new();
    lo_message_add(msg, "ss", "setType", this->getType());
    ret.push_back(msg);


       msg = lo_message_new();
       lo_message_add(msg, "si", "highlight", this->getHighlight());
       ret.push_back(msg);

       msg = lo_message_new();
       lo_message_add(msg, "si", "manipulate", this->getManipulate());
       ret.push_back(msg);
     */

    return ret;
}

} // end of namespace spin

