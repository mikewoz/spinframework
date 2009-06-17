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

#include <osgManipulator/CommandManager>
#include <osgManipulator/TabBoxDragger>
#include <osgManipulator/TabPlaneDragger>
#include <osgManipulator/TabPlaneTrackballDragger>
#include <osgManipulator/TrackballDragger>
#include <osgManipulator/Translate1DDragger>
#include <osgManipulator/Translate2DDragger>
#include <osgManipulator/TranslateAxisDragger>
#include <osgUtil/IntersectionVisitor>


#include "asPointer.h"
#include "asRay.h"
#include "asBasicNode.h"
#include "asSceneManager.h"
#include "osgUtil.h"

using namespace std;

extern pthread_mutex_t pthreadLock;

// *****************************************************************************
// constructor:
asPointer::asPointer (asSceneManager *sceneManager, char *initID) : asReferenced(sceneManager, initID)
{
	this->setName(string(id->s_name) + ".asPointer");
	nodeType = "asPointer";

	//draggerType = "TabPlaneDragger";
	//draggerType = "TrackballDragger";
	//draggerType = "RotateCylinderDragger";
	draggerType = "Grabber";
	
	// create a command manager
	cmdMgr = new osgManipulator::CommandManager;
	
	selection = new osgManipulator::Selection;
	selection->setName("asManipulator.selection");
	
	
	ea = new osgGA::GUIEventAdapter();
	
	doManipulation = false;
	
	grabbedNode = NULL;
	targetNode = NULL;
	
	intersectList.clear();
	
	
	// We need to set up a callback, that computes intersections during scene
	// graph traversal. This should be on the topmost node, so that during node
	// traversal, we update our parameters before anything is drawn.
	this->setUserData( dynamic_cast<osg::Referenced*>(this) );
	this->setUpdateCallback(new asReferenced_callback);
	
	this->setNodeMask(STATSDATA_NODE_MASK); // nodemask info in asGlobals.h
	
}

// *****************************************************************************
// destructor
asPointer::~asPointer()
{

}


void asPointer::callbackUpdate()
{
	
	int i;
	
	// get parentNode:
	osg::ref_ptr<asRay> parentNode = dynamic_cast<asRay*>(this->parent->s_thing);
	
	if (!parentNode.valid())
	{
		//std::cout << "Error: asPointer [" << this->id->s_name << "] must be attached to an asRay node." << std::endl;
		return;
	}

	//osg::Timer_t startTick = osg::Timer::instance()->tick();

	// get line segment start and end points:
	osg::Matrix myMatrix = osg::computeLocalToWorld(this->currentNodePath);
	
	// only proceed if myMatrix has changed since last time
	// NO!, can't do that in case other objects move!
	//if (this->previousMatrix == myMatrix) return;
	
	
	osg::Vec3 start = myMatrix.getTrans();
	osg::Vec3 end = start + ( myMatrix.getRotate() * osg::Vec3(0.0,parentNode->getLength(),0.0) );

	
	
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

	

	
	
	// TODO: dragger event:
	if ((doManipulation) && dragger.valid() & (this->previousMatrix != myMatrix))
	{
		// find change in translation and orientation from original:
		osg::Quat r1 = origMatrix.getRotate();
		osg::Quat r2 = myMatrix.getRotate();					
		osg::Quat dr = r2 - r1;
		
		/*
		osg::Vec3 p1 = origMatrix.getTrans();
		osg::Vec3 p2 = myMatrix.getTrans();
		osg::Vec3 dp = p2 - p1;		
		*/

		osg::Vec3 drVect = Vec3inDegrees(QuatToEuler(dr));
			
		_pointer._hitIter = _pointer._hitList.begin();
		_pointer.setMousePosition(drVect.x(), drVect.z());
			
		ea->setEventType(osgGA::GUIEventAdapter::DRAG);
		dragger->handle(_pointer, *ea.get(), aa);
	}

	
	this->previousMatrix = myMatrix;

	
	
	
	
	
	
	// Store our intersections:
	
	std::vector<t_symbol*> newIntersectList;
	std::vector<osgUtil::LineSegmentIntersector::Intersection> newIntersectData;
	std::vector<osg::Vec3> newItersectListOffsets;
	vector<t_symbol*>::iterator iter;
	
	if ( intersector->containsIntersections() )
	{
		osg::ref_ptr<asReferenced> testNode;
		
		osgUtil::LineSegmentIntersector::Intersections& intersections = intersector->getIntersections();
		osgUtil::LineSegmentIntersector::Intersections::iterator itr;

		for (itr = intersections.begin(); itr != intersections.end(); ++itr)
		{
			const osgUtil::LineSegmentIntersector::Intersection& intersection = *itr;
			for (i=intersection.nodePath.size()-1; i>=0; i--)
			{
				testNode = dynamic_cast<asReferenced*>(intersection.nodePath[i]);
				if (testNode.valid()) // && (testNode->nodeType != "asRay"))
				{
					// only add the hit if not already in the list (note that
					// one node might have several intersections, for each face
					// that instersects with the line segment)
					iter = std::find( newIntersectList.begin(), newIntersectList.end(), testNode->id );
					if ( iter == newIntersectList.end() )
					{
						newIntersectList.push_back(testNode->id);
						newIntersectData.push_back(intersection);
						intersectListOffsets.push_back(intersection.getLocalIntersectPoint());
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
			std::cout<<"  localPoint  ("<<dbgVect.x()<<","<<dbgVect.y()<<","<<dbgVect.z()<<")"<<std::endl;
			dbgVect = intersection.getLocalIntersectNormal();
			std::cout<<"  localNormal ("<<dbgVect.x()<<","<<dbgVect.y()<<","<<dbgVect.z()<<")"<<std::endl;
			dbgVect = intersection.getWorldIntersectPoint();
			std::cout<<"  worldPoint  ("<<dbgVect.x()<<","<<dbgVect.y()<<","<<dbgVect.z()<<")"<<std::endl;
			dbgVect = intersection.getWorldIntersectNormal();
			std::cout<<"  worldNormal ("<<dbgVect.x()<<","<<dbgVect.y()<<","<<dbgVect.z()<<")"<<std::endl;
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
			for (i=0; i<intersectList.size(); i++) lo_message_add_string(msg, (char*)intersectList[i]->s_name);
		} else lo_message_add_string(msg, "NULL");
		
		sceneManager->sendNodeMessage(this->id, msg);
		
	}
	
}
	
// *****************************************************************************


void asPointer::enableDragger()
{
	if (!targetNode.valid()) return;
	
	// if dragger is currently attached somewhere, remove it first:
	if (dragger.valid()) disableDragger();		
	

    if ("TabPlaneDragger" == this->draggerType)
    {
        osgManipulator::TabPlaneDragger* d = new osgManipulator::TabPlaneDragger();
        d->setupDefaultGeometry();
        dragger = d;
    }
    else if ("TabPlaneTrackballDragger" == this->draggerType)
    {
        osgManipulator::TabPlaneTrackballDragger* d = new osgManipulator::TabPlaneTrackballDragger();
        d->setupDefaultGeometry();
        dragger = d;
    }
    else if ("TrackballDragger" == this->draggerType)
    {
        osgManipulator::TrackballDragger* d = new osgManipulator::TrackballDragger();
        d->setupDefaultGeometry();
        dragger = d;
    }
    else if ("Translate1DDragger" == this->draggerType)
    {
        osgManipulator::Translate1DDragger* d = new osgManipulator::Translate1DDragger();
        d->setupDefaultGeometry();
        dragger = d;
    }
    else if ("Translate2DDragger" == this->draggerType)
    {
        osgManipulator::Translate2DDragger* d = new osgManipulator::Translate2DDragger();
        d->setupDefaultGeometry();
        dragger = d;
    }
    else if ("TranslateAxisDragger" == this->draggerType)
    {
        osgManipulator::TranslateAxisDragger* d = new osgManipulator::TranslateAxisDragger();
        d->setupDefaultGeometry();
        dragger = d;
    }
    else if ("TabBoxDragger" == this->draggerType)
    {
        osgManipulator::TabBoxDragger* d = new osgManipulator::TabBoxDragger();
        d->setupDefaultGeometry();
        dragger = d;
    }
    else
    {
    	// NULL dragger
    	return;
    }
	
	pthread_mutex_lock(&pthreadLock);
    
	dragger->setName("asManipulator.dragger");
	dragger->setNodeMask(GEOMETRIC_NODE_MASK); // make sure intersector sees it
	
	targetNode->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
	targetNode->addChild(dragger.get());

	float scale = targetNode->getBound().radius() * 1.1;
	dragger->setMatrix(osg::Matrix::scale(scale, scale, scale) * osg::Matrix::translate(dragger->getBound().center()));
	
	cmdMgr->connect(*dragger.get(), *selection.get());
	
	pthread_mutex_unlock(&pthreadLock);
	
}

void asPointer::disableDragger()
{
	pthread_mutex_lock(&pthreadLock);
	
	if (dragger.valid() && targetNode.valid())
	{
		cmdMgr->disconnect(*dragger.get());
        targetNode->removeChild(dragger.get());
		dragger = NULL;
	}
	
	pthread_mutex_unlock(&pthreadLock);
}


asReferenced *asPointer::getNodeFromIntersections()
{

	if (!intersectList.size()) return NULL;
		
	asReferenced *t = intersectList[0]->s_thing; // intersectList stores t_symbols

	while (t)
	{
		if (t->nodeType == "asBasicNode") 
		{
			return t; // return first basicNode encountered
		}
		t = t->parent->s_thing;
	}
	
	return NULL;
}

/*
void asPointer::computeRT(t_symbol *src, t_symbol *dst, osg::Vec3 &R, osg::Vec3 &T)
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

void asPointer::setType (char *s)
{
	this->draggerType = s;
	
	BROADCAST(this, "ss", "setType", this->getType());
}


void asPointer::highlight (int b)
{

	if (b) {
		this->targetNode = getNodeFromIntersections();
		enableDragger();
	}
	else {
		disableDragger();
		this->targetNode = NULL;
	}
	
	BROADCAST(this, "si", "highlight", this->getHighlight());
}

/**
 * The manipulate() method performs OSG dragger manipulations (once a node has
 * been "highlighted" for manipulation). That is, a dragger needs to be attached
 * to a node somewhere, and we check intersections to see if the pointer has
 * selected any of the dragger handles.
 */
void asPointer::manipulate (int b)
{
	if (sceneManager->isSlave() && !dragger.valid()) return;
	
	pthread_mutex_lock(&pthreadLock);
	

	
	
	// Start manipulation:
	
	// Note, the dragger should have been set by the highlight() method

	
	if (b && intersectList.size() && dragger.valid())
	{
		
		// reset selection:
		selection->setMatrix(osg::Matrix::identity());
		
		// insert the selection node between targetNode and it's parent:
		selection->addChild(targetNode.get());
		targetNode->getParent(0)->replaceChild(targetNode.get(), selection.get());
		
		origMatrix = osg::computeLocalToWorld(this->currentNodePath);
		
		// transfer the intersections to the _pointer
		_pointer.reset();
		_pointer.setCamera(NULL);
		
		osgUtil::LineSegmentIntersector::Intersections& intersections = intersector->getIntersections();
		osgUtil::LineSegmentIntersector::Intersections::iterator hitr;
		for (hitr = intersections.begin(); hitr != intersections.end(); ++hitr)
		{
			_pointer.addIntersection(hitr->nodePath, hitr->getLocalIntersectPoint());
		}

		// set event to PUSH and invoke the dragger:
		ea->setEventType(osgGA::GUIEventAdapter::PUSH);
		dragger->handle(_pointer, *ea.get(), aa);

		doManipulation = true;
	}
	
	// end manipulation:
	else if (dragger.valid())
	{		
		//  set event to RELEASE and invoke handle()

		_pointer._hitIter = _pointer._hitList.begin();
		
		ea->setEventType(osgGA::GUIEventAdapter::RELEASE);
		dragger->handle(_pointer, *ea.get(), aa);

		// update the target's matrix based on selection node, then put the
		// targetNode back on it's parent, and remove selection:
			
		osg::ref_ptr<asBasicNode> targetAsBasicNode = dynamic_cast<asBasicNode*>(targetNode.get());
		if (targetAsBasicNode.valid())
		{
			osg::Matrix m = selection->getMatrix();
			osg::Vec3 mPos = m.getTrans();
			osg::Vec3 mRot = Vec3inDegrees(QuatToEuler(m.getRotate()));
			targetAsBasicNode->setTranslation(mPos.x(), mPos.y(), mPos.z());
			targetAsBasicNode->setOrientation(mRot.x(), mRot.y(), mRot.z());
		}
				
		selection->removeChild(targetNode.get());
		selection->getParent(0)->replaceChild(selection.get(), targetNode.get());
		
		targetNode = NULL;
		doManipulation = false;
	}

	
	pthread_mutex_unlock(&pthreadLock);

	
	BROADCAST(this, "si", "manipulate", this->getManipulate());
}





/* For grabbing, we take the first intersected node, andtemporarily attach it to
 * the pointer, allowing it to inherit any translation or rotation offsets, and
 * then return it to it's previous parent once it is "ungrabbed".
 */
void asPointer::grab (int b)
{
	// return if this VESS is a slave
	if (sceneManager->isSlave()) return;
	
	
	osg::Matrix srcMatrix, dstMatrix;
	

	// start grab:
	if (b && intersectList.size())
	{
		
		// What do we do if a node is already grabbed? let go and grab again?
		// ... for now, let's do nothing.
		if (grabbedNode.valid()) return;
		
		
		grabbedNode = getNodeFromIntersections();
		
		//osg::Vec3 localIntersectPt = intersectListOffsets[0];
		osg::Vec3 localIntersectPt = intersectData[0].getLocalIntersectPoint();
		osg::Vec3 worldIntersectPt = intersectData[0].getWorldIntersectPoint();
		
		
		if (!grabbedNode.valid()) return;
		
		osg::ref_ptr<asBasicNode> n = dynamic_cast<asBasicNode*>(grabbedNode.get());
		
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
		

		/*
		osg::Vec3 iv = ( srcMatrix.getRotate() * (worldIntersectPt-p1) );
		std::cout << "p1=("<<p1.x()<<","<<p1.y()<<","<<p1.z()<< ")"<<std::endl;
		std::cout << "iv=("<<iv.x()<<","<<iv.y()<<","<<iv.z()<< ")"<<std::endl;
		std::cout << "dp=("<<dp.x()<<","<<dp.y()<<","<<dp.z()<< ")"<<std::endl;
		std::cout << "localIntersectPt=("<<localIntersectPt.x()<<","<<localIntersectPt.y()<<","<<localIntersectPt.z()<< ")"<<std::endl;
		std::cout << "worldIntersectPt=("<<worldIntersectPt.x()<<","<<worldIntersectPt.y()<<","<<worldIntersectPt.z()<< ")"<<std::endl;
		std::cout << "world - p1 =     ("<<worldIntersectPt.x()-p1.x()<<","<<worldIntersectPt.y()-p1.y()<<","<<worldIntersectPt.z()-p1.z()<< ")"<<std::endl;
		*/

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
		
		pthread_mutex_lock(&pthreadLock);

			grabbedNode->newParent = this->id;
			grabbedNode->attach();
		
		pthread_mutex_unlock(&pthreadLock);


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
			srcMatrix = osg::computeLocalToWorld(previousParent->s_thing->currentNodePath);

		dstMatrix = osg::computeLocalToWorld(grabbedNode->currentNodePath);	
			
		// find change in translation and orientation from src to dst:
		osg::Vec3 r1 = Vec3inDegrees(QuatToEuler(srcMatrix.getRotate()));
		osg::Vec3 r2 = Vec3inDegrees(QuatToEuler(dstMatrix.getRotate()));		
		osg::Vec3 R = r2 - r1;
				
		osg::Vec3 p1 = srcMatrix.getTrans();
		osg::Vec3 p2 = dstMatrix.getTrans();
		osg::Vec3 T = p2 - p1;			
		
		
		// re-attach node to it's old parent:
		
		pthread_mutex_lock(&pthreadLock);
		
			grabbedNode->newParent = previousParent;
			grabbedNode->attach();
		
		pthread_mutex_unlock(&pthreadLock);
		
		//std::cout << "Re-attaching node [" << targetNode->id->s_name << "] to old parent [" << oldTargetParent->s_name << "] with T=(" <<T.x()<<","<<T.y()<<","<<T.z()<< "), R=(" <<R.x()<<","<<R.y()<<","<<R.z()<< ")" << std::endl;

		// now apply the offset:
		// (TODO: use sceneManager->invokeMethod)
		osg::ref_ptr<asBasicNode> n = dynamic_cast<asBasicNode*>(grabbedNode.get());
		n->setTranslation(T.x(), T.y(), T.z());
		n->setOrientation(R.x(), R.y(), R.z());
		
		grabbedNode = NULL;

	}
	
	BROADCAST(this, "si", "grab", this->getGrab());
}


void asPointer::pull (float f)
{
	if (grabbedNode.valid())
	{
		osg::ref_ptr<asBasicNode> n = dynamic_cast<asBasicNode*>(grabbedNode.get());
		osg::Vec3 T = n->getTranslation() + osg::Vec3(0,f,0);
		
		// (TODO: use sceneManager->invokeMethod)
		n->setTranslation(T.x(), T.y(), T.z());
	}
}


// *****************************************************************************

std::vector<lo_message> asPointer::getState ()
{
	// inherit state from base class
	std::vector<lo_message> ret = asReferenced::getState();
	
	lo_message msg;
	
	msg = lo_message_new();
	lo_message_add(msg, "ss", "setType", this->getType());
	ret.push_back(msg);
	
	/*
	msg = lo_message_new();
	lo_message_add(msg, "si", "highlight", this->getHighlight());
	ret.push_back(msg);
	
	msg = lo_message_new();
	lo_message_add(msg, "si", "manipulate", this->getManipulate());
	ret.push_back(msg);
	*/
	
	return ret;
}
