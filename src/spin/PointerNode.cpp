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

#include "PointerNode.h"

#include <osgManipulator/CommandManager>
#include <osgManipulator/TabBoxDragger>
#include <osgManipulator/TabPlaneDragger>
#include <osgManipulator/TabPlaneTrackballDragger>
#include <osgManipulator/TrackballDragger>
#include <osgManipulator/Translate1DDragger>
#include <osgManipulator/Translate2DDragger>
#include <osgManipulator/TranslateAxisDragger>
#include <osgUtil/IntersectionVisitor>

#include "RayNode.h"
#include "GroupNode.h"
#include "SceneManager.h"
#include "spinApp.h"
#include "spinBaseContext.h"
#include "osgUtil.h"
#include "spinUtil.h"

using namespace std;

extern pthread_mutex_t sceneMutex;

namespace spin
{

// *****************************************************************************
// constructor:
PointerNode::PointerNode (SceneManager *sceneManager, char *initID) : RayNode(sceneManager, initID)
{
    this->setName(string(id->s_name) + ".PointerNode");
    nodeType = "PointerNode";

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

    grabbedNode = NULL;
    targetNode = NULL;

    intersectList.clear();
    
    
    // We use a PointerInfo object to interface with OSG's GUIEventAdapter class
    // Usually PointerInfo is used to construct a linesegment based on mouse x,y
    // projection into 3D space. PointerInfo's method projectWindowXYIntoObject
    // is used to create the line segment, which needs a camera, but we don't
    // need that (since we already have a 3D linesegment).
    _pointer.setCamera(NULL);


    this->setNodeMask(STATSDATA_NODE_MASK); // nodemask info in spinUtil.h

}

// *****************************************************************************
// destructor
PointerNode::~PointerNode()
{

}

void PointerNode::callbackUpdate()
{
    RayNode::callbackUpdate();
    

    // TODO: It's too much work to perform an intersection traversal on all
    // objects in the scenegraph. We should add a way to limit this to only
    // nodes with a certain nodemask so that the user can optimize and choose
    // which parts of the scene should be included / excluded.
    
    //osg::Timer_t startTick = osg::Timer::instance()->tick();

    // get line segment start and end points:
    osg::Matrix myMatrix = osg::computeLocalToWorld(this->currentNodePath);

    // only proceed if myMatrix has changed since last time
    // NO!, can't do that in case other objects move!
    //if (this->previousMatrix == myMatrix) return;

    osg::Vec3 start = myMatrix.getTrans();
    osg::Vec3 end = start + ( myMatrix.getRotate() * osg::Vec3(0.0,this->getLength(),0.0) );





    // create an intersector and create an intersectorVisitor.
    //osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector = new osgUtil::LineSegmentIntersector(start, end);
    intersector = new osgUtil::LineSegmentIntersector(start, end);
    osgUtil::IntersectionVisitor intersectVisitor( intersector.get() );

    // make sure to provide a nodemask so that only geometric nodes are checked
    // for intersections:
    intersectVisitor.setTraversalMask(GEOMETRIC_NODE_MASK);

    // start the visitor:
    sceneManager->worldNode->accept(intersectVisitor);

    //osg::Timer_t endTick = osg::Timer::instance()->tick();
    //std::cout<<"Intersection completed in "<<osg::Timer::instance()->delta_s(startTick,endTick)<<std::endl;


    //if (spinApp::Instance().getContext()->isServer())
    {
        reportIntersections();
    }
    
    applyManipulation(myMatrix, start, end);
     
    this->previousMatrix = myMatrix;
}

void PointerNode::applyManipulation(osg::Matrix mat, osg::Vec3 start, osg::Vec3 end)
{    

    // If the user is holding down the manipulate button, and the dragger has
    // not yet been created, then we need to create a dragger
    if ((doManipulation) && (!dragger.valid()))
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
        std::cout << intersections.size() << " INTERSECTIONS!!!!" << std::endl;
        for (hitr = intersections.begin(); hitr != intersections.end(); ++hitr)
        {
        
            _pointer.addIntersection(hitr->nodePath, hitr->getLocalIntersectPoint());
            
            std::cout << "ADDED intersection to pointer: ";
            for (int i=0; i<hitr->nodePath.size(); i++)
            {
                if (hitr->nodePath[i]->getName().empty()) std::cout << " ?";
                else std::cout << " " << hitr->nodePath[i]->getName();
            }
            std::cout << std::endl;
                
                
            /*
            osg::NodePath cutNodePath;
            
            
            // On the server side, we have an error when using TabBoxDraggers
            // since all Dragger Handles (the green corner squares) are listed
            // in the nodepath. This is NOT the case on the viewer side.
            
            // So here, we make sure to pass only the nodepath up
            // to the first dragger, and then break.
            
            
            
            
            std::cout << "ADDED intersection to pointer: ";
            for (int i=0; i<hitr->nodePath.size(); i++)
            //for (osg::NodePath::iterator itr2 = hitr->nodePath.begin(); itr2 != hitr->nodePath.end(); ++itr2)
            {
                //std::cout << " " << (*itr2)->getName();
                std::cout << " " << hitr->nodePath[i]->getName() <<"("<<hitr->nodePath[i]->getNodeMask() << ")";
                
                
                cutNodePath.push_back(hitr->nodePath[i]);
                
                if (dynamic_cast<osgManipulator::Dragger*>(hitr->nodePath[i]))
                {
                    std::cout << "BREAK" << std::endl;
                    break;
                }
                
            }
            std::cout << std::endl;
            
            
            osg::Node *lastNode = hitr->nodePath[hitr->nodePath.size()-1];
            if (lastNode->getName()!="Dragger Handle")
            {
                std::cout << "ADDED intersection to pointer: ";
                for (int i=0; i<hitr->nodePath.size(); i++)
                    std::cout << " " << hitr->nodePath[i]->getName();
                std::cout << std::endl;
                
                _pointer.addIntersection(hitr->nodePath, hitr->getLocalIntersectPoint());
            } else
            {
            
            }
            
            */
            
       }
        
       
        //_pointer._hitIter = _pointer._hitList.begin();


        
        /*
        for (hitr = intersections.begin(); hitr != intersections.end(); ++hitr)
        {
            std::cout << "intersection:" << std::endl;
            const osgUtil::LineSegmentIntersector::Intersection& intersection = *hitr;
            for (int i=intersection.nodePath.size()-1; i>=0; i--)
            {
                
                osgManipulator::Dragger* dragger = dynamic_cast<osgManipulator::Dragger*>(intersection.nodePath[i]);
                if (dragger)
                {
                    std::cout << i<<") dragger!: " << intersection.nodePath[i]->getName() << std::endl;
                    //break;
                }
                else std::cout << i<<") " << intersection.nodePath[i]->getName() << std::endl;
            }
        }
        */
        
        
        for (osg::NodePath::iterator itr = _pointer._hitList.front().first.begin();
             itr != _pointer._hitList.front().first.end();
             ++itr)
        {
                    
            osgManipulator::Dragger* ptrDragger = dynamic_cast<osgManipulator::Dragger*>(*itr);
            if (ptrDragger)
            {
                std::cout << "YES! manipulator PUSH (dragger name: " << ptrDragger->getName() << "). Intersection:";
                for (osg::NodePath::iterator itr2 = _pointer._hitList.front().first.begin(); itr2 != _pointer._hitList.front().first.end(); ++itr2)
                {
                    std::cout << " " << (*itr2)->getName();
                }
                std::cout << std::endl;

                                                
                                                                                
                if (dynamic_cast<osgManipulator::TabBoxDragger*>(*itr))
                {
                    //_pointer._hitList.front().first.pop_back();
                    
                    //std::cout << "this is h:";
                 
                }


                // Update the PointerInfo by setting the start and end points
                // based on our PointerNode ray:
                _pointer.setNearFarPoints(start,end);
                
                ea->setEventType(osgGA::GUIEventAdapter::PUSH);
                ptrDragger->handle(_pointer, *ea.get(), aa);
                ptrDragger->setDraggerActive(true);
                dragger = ptrDragger;
                //break;
            }
        }
    }

    // if the dragger is already valid, then just do the DRAG
    //else if (doManipulation && dragger.valid())
    else if (doManipulation && dragger.valid() && (this->previousMatrix != mat))
    {
        // Update the PointerInfo by setting the start and end points based on our
        // PointerNode ray:
        _pointer.setNearFarPoints(start,end);
        _pointer._hitIter = _pointer._hitList.begin();

        std::cout << "manipulator DRAG: handled=" << ea->getHandled() << " wxh=" << ea->getWindowWidth()<<"x"<<ea->getWindowHeight() << std::endl;

        //ea->setHandled(false);
        ea->setEventType(osgGA::GUIEventAdapter::DRAG);
        dragger->handle(_pointer, *ea.get(), aa);
    }
    
    // if the dragger is valid and the manipulator flag has been set to off,
    // then we release the dragger:
    else if (!doManipulation && (dragger.valid()))
    {
        //  set event to RELEASE and invoke handle()
        std::cout << "manipulator RELEASE" << std::endl;
        
        _pointer.setNearFarPoints(start,end);
        _pointer._hitIter = _pointer._hitList.begin();
        
        ea->setEventType(osgGA::GUIEventAdapter::RELEASE);
        dragger->handle(_pointer, *ea.get(), aa);
        dragger->setDraggerActive(false);
        _pointer.reset();

        dragger = NULL;
    }

}


    
void PointerNode::reportIntersections()
{

    // Store and report our intersections:

    std::vector<t_symbol*> newIntersectList;
    std::vector<osgUtil::LineSegmentIntersector::Intersection> newIntersectData;
    std::vector<osg::Vec3> newItersectListOffsets;
    vector<t_symbol*>::iterator iter;

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
                    iter = std::find( newIntersectList.begin(), newIntersectList.end(), testNode->id );
                    if ( iter == newIntersectList.end() )
                    {
                        newIntersectList.push_back(testNode->id);
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
        
        if (1)
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
            for (int i=0; i<intersectList.size(); i++) lo_message_add_string(msg, (char*)intersectList[i]->s_name);
        }
        else
            lo_message_add_string(msg, "NULL");
        
        // debug:
        if (1)
        {
            std::cout << "PionterNode has new intersections:";
            if (intersectList.size())
            {
                for (int i=0; i<intersectList.size(); i++)
                    std::cout << " " << (char*)intersectList[i]->s_name;
            } else std::cout << " NULL";
            std::cout << std::endl;
        }
        // end debug

        //sceneManager->sendNodeMessage(this->id, msg);
        if (spinApp::Instance().getContext()->isServer())
            NODE_LO_MSG(sceneManager, this, msg);
    }

}

// *****************************************************************************

ReferencedNode *PointerNode::getNodeFromIntersections()
{

    if (!intersectList.size()) return NULL;

    // we look for the first item in the list that can be cast as a GroupNode
    ReferencedNode *t = dynamic_cast<ReferencedNode*>(intersectList[0]->s_thing); // intersectList stores t_symbols

    while (t)
    {
        GroupNode *g = dynamic_cast<GroupNode*>(t);
        //if (t->nodeType == "GroupNode")
        if (t)
        {
            return t; // return first GroupNode encountered
        }
        t = dynamic_cast<ReferencedNode*>(t->parent->s_thing);
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
        srcMatrix = osg::computeLocalToWorld(src->s_thing->currentNodePath);

    if (dst==gensym("world"))
        dstMatrix = osg::Matrix::identity();
    else
        dstMatrix = osg::computeLocalToWorld(dst->s_thing->currentNodePath);

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

void PointerNode::manipulate (int b)
{
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

        grabbedNode = getNodeFromIntersections();

        //osg::Vec3 localIntersectPt = intersectListOffsets[0];
        osg::Vec3 localIntersectPt = intersectData[0].getLocalIntersectPoint();
        osg::Vec3 worldIntersectPt = intersectData[0].getWorldIntersectPoint();


        if (!grabbedNode.valid()) return;

        osg::ref_ptr<GroupNode> n = dynamic_cast<GroupNode*>(grabbedNode.get());

        // We will temporarily attach targetNode to this pointer, so we
        // need to keep track of the old parent so we can reattach it:
        previousParent = grabbedNode->parent;

        // we need to give grabbedNode an local offset equivalent to
        // it's current difference from this node

        srcMatrix = osg::computeLocalToWorld(this->currentNodePath);
        dstMatrix = osg::computeLocalToWorld(grabbedNode->currentNodePath);

        osg::Vec3 p1 = srcMatrix.getTrans();
        osg::Vec3 p2 = dstMatrix.getTrans();
        osg::Vec3 dp = p2 - p1;


        
        osg::Vec3 iv = ( srcMatrix.getRotate() * (worldIntersectPt-p1) );
        std::cout << "p1=("<<p1.x()<<","<<p1.y()<<","<<p1.z()<< ")"<<std::endl;
        std::cout << "iv=("<<iv.x()<<","<<iv.y()<<","<<iv.z()<< ")"<<std::endl;
        std::cout << "dp=("<<dp.x()<<","<<dp.y()<<","<<dp.z()<< ")"<<std::endl;
        std::cout << "localIntersectPt=("<<localIntersectPt.x()<<","<<localIntersectPt.y()<<","<<localIntersectPt.z()<< ")"<<std::endl;
        std::cout << "worldIntersectPt=("<<worldIntersectPt.x()<<","<<worldIntersectPt.y()<<","<<worldIntersectPt.z()<< ")"<<std::endl;
        std::cout << "world - p1 =     ("<<worldIntersectPt.x()-p1.x()<<","<<worldIntersectPt.y()-p1.y()<<","<<worldIntersectPt.z()-p1.z()<< ")"<<std::endl;
        

        // ARGH. localIntersectPt is not correct!! TODO
        //osg::Vec3 T = osg::Vec3(-localIntersectPt.x(),dp.length(),-localIntersectPt.z());
        osg::Vec3 T = osg::Vec3(0,dp.length(),0);

        osg::Vec3 r1 = Vec3inDegrees(QuatToEuler(srcMatrix.getRotate()));
        osg::Vec3 r2 = Vec3inDegrees(QuatToEuler(dstMatrix.getRotate()));
        osg::Vec3 R = r2 - r1;



        //T = osg::Vec3(0,dp.length(),0);
        //R = osgVec3(0,0,0);

        std::cout << "Attaching node [" << grabbedNode->id->s_name << "] to pointer with T=(" <<T.x()<<","<<T.y()<<","<<T.z()<< "), R=(" <<R.x()<<","<<R.y()<<","<<R.z()<< ")" << std::endl;

        // attach node to this pointer:
        grabbedNode->newParent = this->id;
        grabbedNode->attach(); // this method locks the sceneMutex


        // now apply the offset:
        //
        // TODO: do this with sceneManager->invokeMethod() so that it need not
        //       be a basicNode.


        n->setTranslation(T.x(), T.y(), T.z());
        n->setOrientation(R.x(), R.y(), R.z());
    }

    // end grab:
    else if (grabbedNode.valid())
    {

        // get the global positions of our targetNode and oldTargetParent

        if (previousParent==gensym("world"))
            srcMatrix = osg::Matrix::identity();
        else
            srcMatrix = osg::computeLocalToWorld(dynamic_cast<ReferencedNode*>(previousParent->s_thing)->currentNodePath);

        dstMatrix = osg::computeLocalToWorld(grabbedNode->currentNodePath);

        // find change in translation and orientation from src to dst:
        osg::Vec3 r1 = Vec3inDegrees(QuatToEuler(srcMatrix.getRotate()));
        osg::Vec3 r2 = Vec3inDegrees(QuatToEuler(dstMatrix.getRotate()));
        osg::Vec3 R = r2 - r1;

        osg::Vec3 p1 = srcMatrix.getTrans();
        osg::Vec3 p2 = dstMatrix.getTrans();
        osg::Vec3 T = p2 - p1;


        // re-attach node to it's old parent:
        grabbedNode->newParent = previousParent;
        grabbedNode->attach(); // this method locks the sceneMutex

        std::cout << "Re-attaching node [" << grabbedNode->id->s_name << "] to old parent [" << previousParent->s_name << "] with T=(" <<T.x()<<","<<T.y()<<","<<T.z()<< "), R=(" <<R.x()<<","<<R.y()<<","<<R.z()<< ")" << std::endl;

        // now apply the offset:
        // (TODO: use sceneManager->invokeMethod)
        osg::ref_ptr<GroupNode> n = dynamic_cast<GroupNode*>(grabbedNode.get());
        n->setTranslation(T.x(), T.y(), T.z());
        n->setOrientation(R.x(), R.y(), R.z());

        grabbedNode = NULL;

    }

    BROADCAST(this, "si", "grab", this->getGrab());
}


void PointerNode::slide (float f)
{
    if (grabbedNode.valid())
    {
        osg::ref_ptr<GroupNode> n = dynamic_cast<GroupNode*>(grabbedNode.get());
        osg::Vec3 T = n->getTranslation() + osg::Vec3(0,f,0);

        // (TODO: use sceneManager->invokeMethod)
        n->setTranslation(T.x(), T.y(), T.z());
    }
}


// *****************************************************************************

std::vector<lo_message> PointerNode::getState () const
{
    // inherit state from base class
    std::vector<lo_message> ret = RayNode::getState();

    lo_message msg;

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

