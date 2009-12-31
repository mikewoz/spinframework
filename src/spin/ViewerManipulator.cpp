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

//#include <osgGA/GUIEventAdapter>

#include "ViewerManipulator.h"
#include "osgUtil.h"
#include "spinUtil.h"

using namespace osgGA;

//ViewerManipulator::ViewerManipulator(spinContext* s, UserNode *u)
//ViewerManipulator::ViewerManipulator(UserNode *u)
ViewerManipulator::ViewerManipulator()
{
	spinContext &spin = spinContext::Instance();
	
	if (spin.userNode.valid())
	{
		this->user = spin.userNode->id;
		setTrackNode(spin.userNode->getAttachmentNode());
	} else {
		std::cout << "ERROR: Could not set up node tracker for ViewerManipulator. Perhaps user was registered before SPIN was started?" << std::endl;
	}

	this->raw = false;
	this->picker = false;
	this->mover = true;
	selectedNode=gensym("NULL");
	
	redirectAddr = NULL;
	redirectServ = NULL;

	// set up user node tracker:
	setTrackerMode(  osgGA::NodeTrackerManipulator::NODE_CENTER_AND_ROTATION );
	setRotationMode( osgGA::NodeTrackerManipulator::ELEVATION_AZIM );
	setMinimumDistance( 0.0001 );
	setHomePosition( osg::Vec3(0,-0.0001,0), osg::Vec3(0,0,0), osg::Vec3(0,0,1), false );

}

ViewerManipulator::~ViewerManipulator()
{
	if (redirectServ)
	{
		lo_server_thread_stop(redirectServ);
		usleep(100);
	
		lo_server_thread_free(redirectServ);
	}
}


void ViewerManipulator::setRedirection(std::string addr, std::string port)
{
	//if (redirectServ) lo_server_free(redirectServ);
	
	redirectAddr = lo_address_new(addr.c_str(), port.c_str());
	
	if (isMulticastAddress(addr))
	{
		redirectServ = lo_server_thread_new_multicast(addr.c_str(), NULL, oscParser_error);
	}

	else if (isBroadcastAddress(addr))
	{
		redirectServ = lo_server_thread_new(NULL, oscParser_error);
		int sock = lo_server_get_socket_fd(redirectServ);
		int sockopt = 1;
		setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &sockopt, sizeof(sockopt));
	}

	else {
		redirectServ = lo_server_thread_new(NULL, oscParser_error);
	}

	if (!redirectServ) std::cout << "ERROR (ViewerManipulator) - Could not set redirection. Bad address?: " << addr << ":" << port << std::endl;
	else
	{
		std::cout << "  ViewerManipulator is redirecting picking events to: " << lo_address_get_url(redirectAddr) << std::endl;
	}
	
	lo_server_thread_start(redirectServ);
}

void ViewerManipulator::sendEvent(const char *nodeId, const char *types, ...)
{
	va_list ap;
	va_start(ap, types);
	sendEvent(nodeId, types, ap);
}

void ViewerManipulator::sendEvent(const char *nodeId, const char *types, va_list ap)
{
	spinContext &spin = spinContext::Instance();
		
	lo_message msg = lo_message_new();
	int err = lo_message_add_varargs(msg, types, ap);

	if (0)
	{
		std::cout << "ViewerManipulator sending: " << nodeId << std::endl;
		lo_message_pp(msg);
	}
	
	if (!err)
	{
		
		// TODO: fix this:
		//if (redirectServ) lo_send_message_from(redirectAddr, redirectServ, ("/"+string(nodeId)).c_str(), msg);
		if (redirectAddr) lo_send_message(redirectAddr, ("/"+string(nodeId)).c_str(), msg);
		else spin.NodeMessage(nodeId, msg);
		
	} else {
		std::cout << "ERROR (ViewerManipulator) - could not send message: " << err << std::endl;
	}
}


void ViewerManipulator::setPicker(bool b)
{
	if (1)//(picker!=b)
	{
		this->picker = b;
		if (picker) std::cout << "Mouse picking:\t\tEnabled" << std::endl;
		else std::cout << "Mouse picking:\t\tDisabled" << std::endl;
	}
}

void ViewerManipulator::setMover(bool b)
{
	if (1)//(mover!=b)
	{
		this->mover = b;
		if (mover) std::cout << "Camera motion controls:\tEnabled" << std::endl;
		else std::cout << "Camera motion controls:\tDisabled" << std::endl;	
	}
}

void ViewerManipulator::setRaw(bool b)
{
	if (1)//(picker!=b)
	{
		this->raw = b;
		if (raw) std::cout << "Raw mouse events:\tEnabled" << std::endl;
		else std::cout << "Raw mouse events:\tDisabled" << std::endl;
	}
}

bool ViewerManipulator::handle(const GUIEventAdapter& ea, GUIActionAdapter& aa)
{
	if (ea.getEventType()==GUIEventAdapter::FRAME)
	{
		spinContext &spin = spinContext::Instance();
		
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


void ViewerManipulator::handleKeypress(const GUIEventAdapter& ea)
{
	if (ea.getKey()=='r')
	{
		sendEvent(user->s_name, "sfff", "setOrientation", 0.0, 0.0, 0.0, LO_ARGS_END);
	}
}

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

void ViewerManipulator::handleMouse(osgViewer::View* view, const GUIEventAdapter& ea)
{
	int i;
	osgUtil::LineSegmentIntersector::Intersections intersections;
	osgUtil::LineSegmentIntersector::Intersections::iterator itr;
		    
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


	float scrollX, scrollY;
	if (ea.getEventType()==osgGA::GUIEventAdapter::SCROLL)
	{
		scrollX = ea.getScrollingDeltaX();
		scrollY = ea.getScrollingDeltaY();

		// some devices can't report the delta, so we check if both deltas are
		// zero and in that case, we set the delta to a unit value (1.0) in the 
		// appropriate direction
		if (scrollX==0 && scrollY==0)
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
			}
		}
	}
	
	
	if (0)
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
		}
		std::cout << " buttonMask=" << buttonMask << ", modkeyMask=" << modkeyMask << std::endl;
		std::cout << " selectedNode="<<selectedNode->s_name << ", dXYclick: " << dXclick<<","<<dYclick << std::endl;
	}
	
	
	if (this->picker)
	{
		// This is how the Picker works:
		//
		// A node is "selected" when the user does a PUSH on an GroupNode that
		// has an appropriate interactionMode property.
		//
		// We store the id of the selectedNode, so that we can disable camera
		// motion while an object is selected, and so that only one node can be
		// selected at a time.
		//
		// If a node is currently selected (ie, the user did a PUSH but hasn't
		// done a RELEASE yet, then we send subsequent events to selectedNode.
		// Otherwise, we send to the first intersected node.
		//
		// Whether a node is selected or not, we still want to send a hitPoint
		// (for instance, if the user wants to draw on a node without selecting
		// it)... so we always test for intersections and report the point of
		// intersection (hitPoint). However, it is possible for the user to move
		// the cursor so quickly that a selected node is not intersecting
		// anymore. So if we don't find the selectedNode in the intersections
		// list, we just provide the last hitPoint for the event.
		
		bool haveIntersections = view->computeIntersections(ea.getX(),ea.getY(),intersections);

		// first, we fill 2 vectors with data (the pointer to the node, and the
		// local intersect point)
		vector<GroupNode*> hitNodes;
		vector<osg::Vec3> hitPoints;

		// intersections are ordered from nearest to furthest, so we iterate and
		// a return list of all nodes that can be cast as interactive SPIN nodes
		osgUtil::LineSegmentIntersector::Intersections::iterator itr;
		for (itr = intersections.begin(); itr != intersections.end(); ++itr)
		{
			for (i=(*itr).nodePath.size()-1; i>=0; i--)
			{
				osg::ref_ptr<GroupNode> testNode = dynamic_cast<GroupNode*>((*itr).nodePath[i]);
				if (testNode.valid() && (testNode->getInteractionMode()>0))
				{
					// but only add it once (the intersector can intersect with the
					// same node several times:
					if (std::find( hitNodes.begin(), hitNodes.end(), testNode.get() ) == hitNodes.end())
					{
						hitPoints.push_back((*itr).getLocalIntersectPoint());
						hitNodes.push_back(testNode.get());
					}
					// found an interactive SPIN node, so we can break and go
					// to next intersection
					break;
				}
			}
		}


		// for DRAG and PUSH interactions, we are only interested in one node,
		// the "hitNode"
		osg::ref_ptr<GroupNode> hitNode;
			
		// if a node has previously been selected, the hitNode is set to that
		// node. We also see if it was in the intersection list, so that we can
		// update the hitPoint:
		if (selectedNode!=gensym("NULL"))
		{
			hitNode = dynamic_cast<GroupNode*>(selectedNode->s_thing);
			for (i=0; i<hitNodes.size(); i++)
			{
				if (hitNodes[i] == hitNode.get())
				{
					lastHitPoint = hitPoints[i];
					break;
				}
			}
		}
		
		
		// If no node is selected, then the hitNode is set to the first in the
		// list
		else if (hitNodes.size())
	    {
			hitNode = hitNodes[0];
			lastHitPoint = hitPoints[0];
		}
		
		// So now we should have a hitNode to work on and the lastHitPoint		
		if (hitNode.valid())				
		{
			if (( hitNode->getInteractionMode()==GroupNode::DRAG) ||
			    ( hitNode->getInteractionMode()==GroupNode::PUSH))
			{
				switch(ea.getEventType())
				{
					// send MOVE events (eg, for drawing on nodes)
					case(GUIEventAdapter::MOVE):
						sendEvent(hitNode->id->s_name,
								  "sisfffff",
								  "event",
								  (int)ea.getEventType(),
								  user->s_name,
								  dX,
								  dY,
								  lastHitPoint.x(),
								  lastHitPoint.y(),
								  lastHitPoint.z(),
								  LO_ARGS_END);
						break;
					
					// DRAG will only occur if someone clicks elsewhere
					// and rolls onto this node, so we shouldn't send
					// anything, should we?
					case(GUIEventAdapter::DRAG):
						sendEvent(hitNode->id->s_name,
								  "sisfffff",
								  "event",
								  (int)ea.getEventType(),
								  user->s_name,
								  dX,
								  dY,
								  lastHitPoint.x(),
								  lastHitPoint.y(),
								  lastHitPoint.z(),
								  LO_ARGS_END);
						break;
					
					// Same goes for the case where someone rolls onto a
					// node with the mouse button down, and releases.
					// However, just to be safe, we'll also ensure that
					// there is no selected node anymore:
					case(GUIEventAdapter::RELEASE):
						sendEvent(hitNode->id->s_name,
								"sisfffff",
								"event",
								(int)ea.getEventType(),
								user->s_name,
								(float) ea.getModKeyMask(),
								(float) ea.getButtonMask(),
								lastHitPoint.x(),
								lastHitPoint.y(),
								lastHitPoint.z(),
								LO_ARGS_END);
						selectedNode = gensym("NULL");
						break;
					
					// SCROLLING (with the mouse wheel) is cool. It
					// could be used to scale for example.
					case(GUIEventAdapter::SCROLL):
						sendEvent(hitNode->id->s_name,
								  "sisfffff",
								  "event",
								  (int)ea.getEventType(),
								  user->s_name,
								  scrollX,
								  scrollY,
								  lastHitPoint.x(),
								  lastHitPoint.y(),
								  lastHitPoint.z(),
								  LO_ARGS_END);
						break;
					
					// Finally, in the case of a PUSH, we both send the
					// event, and set the selectedNode
					case(GUIEventAdapter::PUSH):
						sendEvent(hitNode->id->s_name,
								  "sisfffff",
								  "event",
								  (int)ea.getEventType(),
								  user->s_name,
								  (float) ea.getModKeyMask(),
								  (float) ea.getButtonMask(),
								  lastHitPoint.x(),
								  lastHitPoint.y(),
								  lastHitPoint.z(),
								  LO_ARGS_END);
						selectedNode = hitNode->id;
						break;
				} // switch
			} // if ((mode==GroupNode::DRAG) || (mode==GroupNode::PUSH))

			else if (hitNode->getInteractionMode()==GroupNode::DRAW)
			{
				switch(ea.getEventType())
				{
					case(GUIEventAdapter::DRAG):
						sendEvent(hitNode->id->s_name,
							"sisfffff",
							"event",
							(int)ea.getEventType(),
							user->s_name,
							dX,
							dY,
							lastHitPoint.x(),
							lastHitPoint.y(),
							lastHitPoint.z(),
							LO_ARGS_END);
						break;

					case(GUIEventAdapter::PUSH):
						sendEvent(hitNode->id->s_name,
							"sisfffff",
							"event",
							(int)ea.getEventType(),
							user->s_name,
							(float) ea.getModKeyMask(),
							(float) ea.getButtonMask(),
							lastHitPoint.x(),
							lastHitPoint.y(),
							lastHitPoint.z(),
							LO_ARGS_END);
						break;
				}
			}
			
		} // if hitNode.valid()
	} // end picker
	
    // scene event processing (eg, camera motion):
	if (this->mover && (selectedNode==gensym("NULL")))
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
