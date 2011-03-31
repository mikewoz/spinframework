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

#include "ViewerManipulator.h"
#include <string>
#include <vector>
#include <osgViewer/View>
#include "osgUtil.h"
#include "spinUtil.h"
#include "spinApp.h"
#include "SceneManager.h"

//using namespace osgGA;

namespace spin
{

//ViewerManipulator::ViewerManipulator(spinContext* s, UserNode *u)
//ViewerManipulator::ViewerManipulator(UserNode *u)
ViewerManipulator::ViewerManipulator()
{
	spinApp &spin = spinApp::Instance();
	
	if (spin.userNode.valid())
	{
		this->user = spin.userNode->id;
		setTrackNode(spin.userNode->getCameraAttachmentNode());
	} else {
		std::cout << "ERROR: Could not set up node tracker for ViewerManipulator. Perhaps user was registered before SPIN was started?" << std::endl;
	}

	this->raw = false;
	this->picker = false;
	this->mover = true;	
	
	// set up user node tracker:
	setTrackerMode(  osgGA::NodeTrackerManipulator::NODE_CENTER_AND_ROTATION );
	//setTrackerMode(  osgGA::NodeTrackerManipulator::NODE_CENTER );
	setRotationMode( osgGA::NodeTrackerManipulator::ELEVATION_AZIM );
	//setMinimumDistance( 0.0001 );
	setMinimumDistance( 0.0 );
	setHomePosition( osg::Vec3(0,-0.001,0), osg::Vec3(0,0,0), osg::Vec3(0,0,1), false );
	//setHomePosition( osg::Vec3(0,0,0), osg::Vec3(0,0.0001,0), osg::Vec3(0,0,1), false );

}

ViewerManipulator::~ViewerManipulator()
{
}


void ViewerManipulator::setPicker(bool b)
{
	this->picker = b;
	if (picker) std::cout << "  Mouse picking:\t\tEnabled" << std::endl;
	else std::cout << "  Mouse picking:\t\tDisabled" << std::endl;
}

void ViewerManipulator::setMover(bool b)
{
	this->mover = b;
	if (mover) std::cout << "  Camera motion controls:\tEnabled" << std::endl;
	else std::cout << "  Camera motion controls:\tDisabled" << std::endl;	
}

void ViewerManipulator::setRaw(bool b)
{
	this->raw = b;
	if (raw) std::cout << "  Raw mouse events:\tEnabled" << std::endl;
	else std::cout << "  Raw mouse events:\tDisabled" << std::endl;
}

bool ViewerManipulator::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
    using namespace osgGA;

	if (ea.getEventType()==GUIEventAdapter::FRAME)
	{
		spinApp &spin = spinApp::Instance();
		
		// update from NodeTrackerManipulator:
		if (spin.userNode.valid())
		{
			// if the userNode's nodepath has changed, we must call setTrackNode
			// again to force NodeTrackerManipulator to store the proper nodePath
			if (spin.userNode->nodepathUpdate)
			{
				setTrackNode(spin.userNode->getAttachmentNode());
				spin.userNode->nodepathUpdate = false;
			}
			
			// update camera from NodeTrackerManipulator:
			if (_thrown) aa.requestRedraw();
		}
	}
	
	else if ((ea.getEventType()==GUIEventAdapter::MOVE)||
			(ea.getEventType()==GUIEventAdapter::DRAG)||
			(ea.getEventType()==GUIEventAdapter::PUSH)||
			(ea.getEventType()==GUIEventAdapter::RELEASE)||
			(ea.getEventType()==GUIEventAdapter::DOUBLECLICK)||
			(ea.getEventType()==GUIEventAdapter::SCROLL))
	{
		osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
		if (view) handleMouse(view,ea);
	}
	
	
	else if (ea.getEventType()==GUIEventAdapter::KEYUP)
	{
		handleKeypress(ea);
	}
	
	return false;
}


void ViewerManipulator::handleKeypress(const osgGA::GUIEventAdapter& ea)
{
	if (ea.getKey()=='r')
	{
		sendEvent(user->s_name, "sfff", "setOrientation", 0.0, 0.0, 0.0, LO_ARGS_END);
	}
}

/*
GroupNode* ViewerManipulator::getNodeFromIntersection(osgUtil::LineSegmentIntersector::Intersection intersection)
{
    osg::ref_ptr<GroupNode> testNode;

    // intersections are ordered from nearest to furthest, so we
    // iterate through and return the first intersection that can
    // be cast as an interactive SPIN node
    for (int i=intersection.nodePath.size()-1; i>=0; i--)
    {
        testNode = dynamic_cast<GroupNode*>(intersection.nodePath[i]);
        if (testNode.valid() && (testNode->getInteractionMode()))
        {
            return testNode.get();
        }
    }

    return NULL;
}

std::vector<GroupNode*> ViewerManipulator::getNodesFromIntersections(osgUtil::LineSegmentIntersector::Intersections intersections)
{
	vector<GroupNode*> nodes;

	// intersections are ordered from nearest to furthest, so we iterate and
	// a return list of all nodes that can be cast as interactive SPIN nodes

	osgUtil::LineSegmentIntersector::Intersections::iterator itr;
	for (itr = intersections.begin(); itr != intersections.end(); ++itr)
	{
		for (int i=(*itr).nodePath.size()-1; i>=0; i--)
		{
			osg::ref_ptr<GroupNode> testNode = dynamic_cast<GroupNode*>((*itr).nodePath[i]);
			if (testNode.valid() && (testNode->getInteractionMode()))
			{
				// but only add it once (the intersector can intersect with the
				// same node several times:
				if (std::find( nodes.begin(), nodes.end(), testNode.get() ) == nodes.end())
					nodes.push_back(testNode.get());
				break;
			}
		}
	}
	
	return nodes;
}
*/

void ViewerManipulator::handleMouse(osgViewer::View* view, const osgGA::GUIEventAdapter& ea)
{
    using namespace osgGA;

	osg::ref_ptr<GroupNode> hitNode, drawNode;
	
	float dX = lastX - ea.getXnormalized();
	float dY = lastY - ea.getYnormalized();

	float dXclick = clickX - ea.getXnormalized();
	float dYclick = clickY - ea.getYnormalized();

	unsigned int buttonMask = ea.getButtonMask();
	unsigned int modkeyMask = ea.getModKeyMask();

	// adjust modkeyMask to ignore numlock and capslock:
	if (modkeyMask>=GUIEventAdapter::MODKEY_CAPS_LOCK) modkeyMask -= GUIEventAdapter::MODKEY_CAPS_LOCK;
	if (modkeyMask>=GUIEventAdapter::MODKEY_NUM_LOCK) modkeyMask -= GUIEventAdapter::MODKEY_NUM_LOCK;
	
	
	// correct dX for aspect ratio:		
	//std::cout << "aspect= " << (float)ea.getWindowWidth()/ea.getWindowHeight() << std::endl;
	dX *= (float)ea.getWindowWidth()/ea.getWindowHeight();
	dXclick *= (float)ea.getWindowWidth()/ea.getWindowHeight();

#if 0
	if (0) // (ea.getEventType() != osgGA::GUIEventAdapter::MOVE)
	{
		switch(ea.getEventType())
		{
			case(osgGA::GUIEventAdapter::PUSH):
				std::cout << "PUSH ("<<ea.getEventType()<<")"; break;
			case(osgGA::GUIEventAdapter::RELEASE):
				std::cout << "RELEASE ("<<ea.getEventType()<<")"; break;
			case(osgGA::GUIEventAdapter::DOUBLECLICK):
				std::cout << "DOUBLECLICK ("<<ea.getEventType()<<")"; break;
			case(osgGA::GUIEventAdapter::DRAG):
				std::cout << "DRAG ("<<ea.getEventType()<<")"; break;
			case(osgGA::GUIEventAdapter::MOVE):
				std::cout << "MOVE ("<<ea.getEventType()<<")"; break;
			case(osgGA::GUIEventAdapter::SCROLL):
				std::cout << "SCROLL ("<<ea.getEventType()<<")"; break;
			default:
				std::cout << "some other message?" << std::endl; break;
		}
		std::cout << " buttonMask=" << buttonMask << ", modkeyMask=" << modkeyMask << ", dXYclick: " << dXclick<<","<<dYclick << std::endl;
		std::cout << " currently selected nodes:";
		for (unsigned j = 0; j < selectedNodes.size(); ++j)
		{
			std::cout << " " << selectedNodes[j]->s_name;
		}
		std::cout << std::endl;
	}
#endif


	float scrollX = 0.0;
    float scrollY = 0.0;
	if (ea.getEventType()==osgGA::GUIEventAdapter::SCROLL)
	{
		scrollX = ea.getScrollingDeltaX();
		scrollY = ea.getScrollingDeltaY();

		// some devices can't report the delta, so we check if both deltas are
		// zero and in that case, we set the delta to a unit value (1.0) in the 
		// appropriate direction
		if (scrollX == 0 && scrollY == 0)
		{
			switch (ea.getScrollingMotion())
			{
				case osgGA::GUIEventAdapter::SCROLL_LEFT:
					scrollX = 1.0;
					break;
				case osgGA::GUIEventAdapter::SCROLL_RIGHT:
					scrollX = -1.0;
					break;
				case osgGA::GUIEventAdapter::SCROLL_UP:
					scrollY = -1.0;
					break;
				case osgGA::GUIEventAdapter::SCROLL_DOWN:
					scrollY = 1.0;
					break;
				default:
					// nothing
					break;
			}
		}
	}
	
	if (this->picker)
	{
		// This is how the Picker works:
		//
		// A node is "selected" when the user does a PUSH on an GroupNode that
		// has an appropriate interactionMode property (eg, DRAG, THROW)
		//
		// We store the id of the selectedNodes, so that we can disable camera
		// motion if anything is selected.
		//
		// Morover, it is possible for the user to move the cursor so quickly
		// that a selected node is not intersecting anymore and we won't find
		// the node in the intersections list. However, SPIN expects us to send
		// a RELEASE event for that node, so the id must be stored.

		osgUtil::LineSegmentIntersector::Intersections intersections;
		view->computeIntersections(ea.getX(),ea.getY(),intersections);
		//bool haveIntersections = view->computeIntersections(ea.getX(),ea.getY(),intersections, INTERACTIVE_NODE_MASK);

		// first, we fill 2 vectors with data (the pointer to the node, and the
		// local intersect point)
        std::vector<GroupNode*> hitNodes;
        std::vector<osg::Vec3> hitPoints;

		// intersections are ordered from nearest to furthest, so we iterate and
		// a return list of all nodes that can be cast as interactive SPIN nodes
		osgUtil::LineSegmentIntersector::Intersections::const_iterator itr;
		for (itr = intersections.begin(); itr != intersections.end(); ++itr)
		{
			// look down the nodePath for the first SPIN node:
			for (int i = (*itr).nodePath.size() - 1; i >= 0; i--)
			{
				osg::ref_ptr<GroupNode> testNode = dynamic_cast<GroupNode*>((*itr).nodePath[i]);
				if (testNode.valid())
				{
					// we check if this node is interactive, or if it has an
					// owner (since sometimes interactive mode can be unset
					// before the RELEASE event gets sent)
					//if ((testNode->getInteractionMode()>0) || (testNode->owner.valid()))
					if (testNode->getInteractionMode()>0)
					{
						// Yes. This is an interactive SPIN node, so add it to 
						// the list, but only once! ... we must check if it has
						// already been added since the intersector can
						// intersect with the same node several times:
						if (std::find( hitNodes.begin(), hitNodes.end(), testNode.get() ) == hitNodes.end())
						{
							osg::Vec3 v = (*itr).getLocalIntersectPoint();
							hitPoints.push_back((*itr).getLocalIntersectPoint());
							hitNodes.push_back(testNode.get());
						}
					}

					// Once we've found the first SPIN node, we break and move
					// to the next intersection. We don't look up the nodePath
					// to see if parentNodes are interactive, otherwise we'll
					// get bad intersect points.
					break;
				}
			}
		}

		// ******
	
		// If any nodes are currently "selected" (ie, the user did a PUSH but
		// hasn't done a RELEASE yet), we are ONLY interested in subsequent
		// events to those node(s). ie, we do not select additional nodes until
		// a RELEASE has been performed.
		//
		// We do however try to use the intersect list to update the hitPoint.
		
		if (selectedNodes.size())
		{
			//for (j=0; j<selectedNodes.size(); j++)
			for (std::vector<t_symbol*>::iterator it=selectedNodes.begin(); it!=selectedNodes.end();)
			{	
				bool found = false;
				GroupNode *n = dynamic_cast<GroupNode*>((*it)->s_thing);
				for (unsigned i = 0; i < hitNodes.size(); i++)
				{
					if (hitNodes[i] == n)
					{
						sendPick(n, ea.getEventType(), modkeyMask, buttonMask,
								scrollX, scrollY, dX, dY, hitPoints[i]);
						found = true;
						break;
					}
				}
				
				// if it wasn't found in the intersections list, we still want to
				// send the event (eg, a RELEASE), but we set the hitpoint to 0,0,0
				if (!found)
				{
					sendPick(n, ea.getEventType(), modkeyMask, buttonMask,
							scrollX, scrollY, dX, dY, osg::Vec3(0.0,0.0,0.0));
				}

				// if the event is a RELEASE, then remove it from selectedNodes
				if (ea.getEventType()==GUIEventAdapter::RELEASE)
					selectedNodes.erase(it);
				else
					++it;
			}
		}
		
		// if no nodes are selected, then we go through the intersect list and
		// send events to the first selectable (ie, type SELECT, DRAG, or THROW)
		// in the list, and the first DRAW node. We NEVER send to more than one
		// selectable, or we will end up selecting several nodes at once.
		//
		// Also, the rule with DRAW nodes is that once we find one, we no longer
		// look for selectables. ie, a selectable node can be selected in front
		// of a drawable (eg, for a brush), but we cannot select nodes behind
		// a drawable.
		else
		{	
			bool foundSelectable = false;
			bool foundDrawable = false;
			
			for (unsigned i = 0; i < hitNodes.size(); i++)
			{
				if ( hitNodes[i]->getInteractionMode()==GroupNode::DRAW )
				{
					if (!foundDrawable)
					{
						sendPick(hitNodes[i], ea.getEventType(), modkeyMask, buttonMask,
								scrollX, scrollY, dX, dY, hitPoints[i]);
						
						// add it to the selectedNodes in the case of a PUSH
						if (ea.getEventType()==GUIEventAdapter::PUSH)
							selectedNodes.push_back(hitNodes[i]->id);
						
						foundDrawable = true;
					}
				}

				// If we've found a drawable, thus we stop right away so that
				// nodes behind cannot be selected
				if (foundDrawable) break;
				
				else if ((int) hitNodes[i]->getInteractionMode()>0)
				{
					if (!foundSelectable)
					{
						sendPick(hitNodes[i], ea.getEventType(), modkeyMask, buttonMask,
								scrollX, scrollY, dX, dY, hitPoints[i]);
						
						// add it to the selectedNodes in the case of a PUSH
						if (ea.getEventType()==GUIEventAdapter::PUSH)
							selectedNodes.push_back(hitNodes[i]->id);
						
						foundSelectable = true;
					}
				}
				
				// if we've found one of each, we can stop:
				if (foundSelectable && foundDrawable) break;

				// otherwise we keep looking, allowing one selectable in front
				// of one drawable
			}
		}

		
	} // end picker
	
	
	
    // scene event processing (eg, camera motion):
	if ( this->mover && selectedNodes.empty() )
	{
		float movScalar = 10.0;
		float rotScalar = 30.0;
		
		switch(ea.getEventType())
		{
			case(GUIEventAdapter::PUSH):
				// in the case of a mouse click, we store the current coords
				clickX = ea.getXnormalized();
				clickY = ea.getYnormalized();
				
				
				sendEvent(user->s_name, "sfff", "setVelocity", 0.0, 0.0, 0.0, LO_ARGS_END);
				if (buttonMask == (GUIEventAdapter::LEFT_MOUSE_BUTTON+GUIEventAdapter::RIGHT_MOUSE_BUTTON))
					sendEvent(user->s_name, "sfff", "setSpin", 0.0, 0.0, 0.0, LO_ARGS_END);

				
				break;
				
			case(GUIEventAdapter::DRAG):
				
				// here we can move the user relative to the current camera
				// view, so we get the view's world coords:
				/*
			    osg::Vec3d camCenter;
			    osg::Quat camRotation;
			    computeNodeCenterAndRotation(camCenter, camRotation);
			    osg::Vec3 camEulers = QuatToEuler( camRotation );
			    float camDistance = ((*itr).getWorldIntersectPoint() - camCenter).length();
			    
			    std::cout << "cam center=("<<camCenter.x()<<","<<camCenter.y()<<","<<camCenter.z()<<")";
			    std::cout << ", distance="<<camDistance;
			    std::cout << ", pitch="<<osg::RadiansToDegrees(camEulers.x());
			    std::cout << ", yaw="<<osg::RadiansToDegrees(camEulers.z());
			    std::cout << std::endl;
			    */
				
				if (!modkeyMask)
				{
				    if (buttonMask == GUIEventAdapter::LEFT_MOUSE_BUTTON)
				    {
				    	// pan forward/back & left/right:
				    	sendEvent(user->s_name, "sfff", "move", dX*movScalar, dY*movScalar, 0.0, LO_ARGS_END);
				    }
				    else if (buttonMask == GUIEventAdapter::RIGHT_MOUSE_BUTTON)
				    {
				    	// pan up/down & left/right:
				    	sendEvent(user->s_name, "sfff", "move", dX*movScalar, 0.0, dY*movScalar, LO_ARGS_END);
				    }
				    else if (buttonMask == (GUIEventAdapter::LEFT_MOUSE_BUTTON+GUIEventAdapter::RIGHT_MOUSE_BUTTON))
				    {
				    	// rotate mode:
				    	sendEvent(user->s_name, "sfff", "rotate", dY*rotScalar, 0.0, dX*rotScalar, LO_ARGS_END);
				    }
				}
			
			    else if ( (modkeyMask==GUIEventAdapter::MODKEY_LEFT_CTRL) || (modkeyMask==GUIEventAdapter::MODKEY_RIGHT_CTRL) )
			    	
			    {
			    	int dXsign, dYsign;
			    	(dXclick<0) ? dXsign=-1 : dXsign=1;
			    	(dYclick<0) ? dYsign=-1 : dYsign=1;
			    	
			    	// drive mode:
				    if (buttonMask == GUIEventAdapter::LEFT_MOUSE_BUTTON)
				    {
				    	// drive forward/back & left/right:
			    		sendEvent(user->s_name, "sfff", "setVelocity", dXsign*pow(dXclick*movScalar,2), dYsign*pow(dYclick*movScalar,2), 0.0f, LO_ARGS_END);
			    		
				    }
				    else if (buttonMask == GUIEventAdapter::RIGHT_MOUSE_BUTTON)
				    {
				    	// drive up/down & left/right:
			    		sendEvent(user->s_name, "sfff", "setVelocity", dXsign*pow(dXclick*movScalar,2), 0.0f, dYsign*pow(dYclick*movScalar,2), LO_ARGS_END);
			    	}
				    else if (buttonMask == (GUIEventAdapter::LEFT_MOUSE_BUTTON+GUIEventAdapter::RIGHT_MOUSE_BUTTON))
				    {
				    	// rotate mode:
			    		sendEvent(user->s_name, "sfff", "setSpin", dYsign*pow(dYclick*rotScalar,2), 0.0, dXsign*pow(dXclick*rotScalar,2), LO_ARGS_END);
				    }
			    }

				break;
				
			case(GUIEventAdapter::RELEASE):
				sendEvent(user->s_name, "sfff", "setVelocity", 0.0, 0.0, 0.0, LO_ARGS_END);
				sendEvent(user->s_name, "sfff", "setSpin", 0.0, 0.0, 0.0, LO_ARGS_END);
				break;
				
			case(GUIEventAdapter::SCROLL):
				// zoom?
				break;
			default:
				// here we could pop up an HUD menu if someone double-clicks or right-clicks
				break;
		}
	}

	// send raw events if user requests them
	if (this->raw)
	{
		sendEvent(user->s_name,
		    "siiiff",
		    "mouseEvent",
			(int)ea.getEventType(),
			(int) ea.getModKeyMask(),
			(int) ea.getButtonMask(),
		    (float) ea.getXnormalized(),
		    (float) ea.getYnormalized(),
		    LO_ARGS_END);
	}

	lastX = ea.getXnormalized();
	lastY = ea.getYnormalized();
}

void ViewerManipulator::sendPick(GroupNode *hitNode, unsigned int eventType, unsigned int modKeyMask, unsigned int buttonMask, float scrollX, float scrollY, float dX, float dY, osg::Vec3 hitPoint)
{

	switch (eventType)
	{
		// send MOVE events (eg, for drawing on nodes)
		case(osgGA::GUIEventAdapter::MOVE):
			sendEvent(hitNode->id->s_name,
					  "sisfffff",
					  "event",
					  (int)eventType,
					  user->s_name,
					  dX,
					  dY,
					  hitPoint.x(),
					  hitPoint.y(),
					  hitPoint.z(),
					  LO_ARGS_END);
			break;
		
		// DRAG will only occur if someone clicks elsewhere
		// and rolls onto this node, so we shouldn't send
		// anything, should we?
		case(osgGA::GUIEventAdapter::DRAG):
			sendEvent(hitNode->id->s_name,
					  "sisfffff",
					  "event",
					  (int)eventType,
					  user->s_name,
					  dX,
					  dY,
					  hitPoint.x(),
					  hitPoint.y(),
					  hitPoint.z(),
					  LO_ARGS_END);
			break;
		
		// Same goes for the case where someone rolls onto a
		// node with the mouse button down, and releases.
		// However, just to be safe, we'll also ensure that
		// there is no selected node anymore:
		case(osgGA::GUIEventAdapter::RELEASE):
			sendEvent(hitNode->id->s_name,
					"sisfffff",
					"event",
					(int)eventType,
					user->s_name,
					(float) modKeyMask,
					(float) buttonMask,
					hitPoint.x(),
					hitPoint.y(),
					hitPoint.z(),
					LO_ARGS_END);
			/*
			for (std::vector<t_symbol*>::iterator it=selectedNodes.begin(); it!=selectedNodes.end(); ++it)
			{
				if ((*it)==hitNode->id)
				{
					selectedNodes.erase(it);
					//break;
				}
			}
			 */
			break;
		
		// SCROLLING (with the mouse wheel) is cool. It
		// could be used to scale for example.
		case(osgGA::GUIEventAdapter::SCROLL):
			sendEvent(hitNode->id->s_name,
					"sisfffff",
					"event",
					(int)eventType,
					user->s_name,
					scrollX,
					scrollY,
					hitPoint.x(),
					hitPoint.y(),
					hitPoint.z(),
					LO_ARGS_END);
			break;
		
		// In the case of a PUSH, we both send the
		// event, and set the selectedNode
		case(osgGA::GUIEventAdapter::PUSH):
			sendEvent(hitNode->id->s_name,
					"sisfffff",
					"event",
					(int)eventType,
					user->s_name,
					(float) modKeyMask,
					(float) buttonMask,
					hitPoint.x(),
					hitPoint.y(),
					hitPoint.z(),
					LO_ARGS_END);
			//selectedNodes.push_back(hitNode->id);
			break;

		case(osgGA::GUIEventAdapter::DOUBLECLICK):
			sendEvent(hitNode->id->s_name,
					"sisfffff",
					"event",
					(int)eventType,
					user->s_name,
					(float) modKeyMask,
					(float) buttonMask,
					hitPoint.x(),
					hitPoint.y(),
					hitPoint.z(),
					LO_ARGS_END);
			break;	
	}
}


void ViewerManipulator::sendEvent(const char *nodeId, const char *types, ...)
{
	va_list ap;
	va_start(ap, types);
	sendEvent(nodeId, types, ap);
}

void ViewerManipulator::sendEvent(const char *nodeId, const char *types, va_list ap)
{
	spinApp &spin = spinApp::Instance();
		
	lo_message msg = lo_message_new();
	int err = lo_message_add_varargs(msg, types, ap);

	if (!err)
		spin.NodeMessage(nodeId, msg);
	else 
		std::cout << "ERROR (ViewerManipulator) - could not send message: " << err << std::endl;
}

} // end of namespace spin


