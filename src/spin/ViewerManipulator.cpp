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
ViewerManipulator::ViewerManipulator(UserNode *u)
{
	//this->spin = s;
	this->user = u;
	this->picker = false;
	this->mover = true;
	selectedNode=gensym("NULL");
	
	redirectServ = NULL;
	
	// defaults:
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

	if (!err)
	{
		if (redirectServ) lo_send_message_from(redirectAddr, redirectServ, ("/"+string(nodeId)).c_str(), msg);
		else spin.sendNodeMessage(nodeId, msg);
		
		//spin.sendNodeMessage(nodeId, msg);
		
	} else {
		std::cout << "ERROR (ViewerManipulator) - could not send message: " << err << std::endl;
	}
}


void ViewerManipulator::setPicker(bool b)
{
	if (picker!=b)
	{
		this->picker = b;
		if (picker) std::cout << "Enabled mouse picking" << std::endl;
		else std::cout << "Disabled mouse picking" << std::endl;
	}
}

void ViewerManipulator::setMover(bool b)
{
	if (mover!=b)
	{
		this->mover = b;
		if (mover) std::cout << "Enabled camera motion controls" << std::endl;
		else std::cout << "Disabled camera motion controls" << std::endl;	
	}
}

bool ViewerManipulator::handle(const GUIEventAdapter& ea, GUIActionAdapter& aa)
{
	if (ea.getEventType()==GUIEventAdapter::FRAME)
	{
		// update from NodeTrackerManipulator:
		if (_thrown) aa.requestRedraw();
	}
	
	else if ((ea.getEventType()==GUIEventAdapter::MOVE)||
			(ea.getEventType()==GUIEventAdapter::DRAG)||
			(ea.getEventType()==GUIEventAdapter::PUSH)||
			(ea.getEventType()==GUIEventAdapter::RELEASE)||
			(ea.getEventType()==GUIEventAdapter::DOUBLECLICK)||
			(ea.getEventType()==GUIEventAdapter::SCROLL))
	{
		osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
		if (view) processEvent(view,ea);
	}
	
	
	else if (ea.getEventType()==GUIEventAdapter::KEYUP)
	{
		processKeypress(ea);
	}
	
	return false;
}


void ViewerManipulator::processKeypress(const GUIEventAdapter& ea)
{
	if (ea.getKey()=='r')
	{
		sendEvent(user->id->s_name, "sfff", "setOrientation", 0.0, 0.0, 0.0, LO_ARGS_END);
	}
}

void ViewerManipulator::processEvent(osgViewer::View* view, const GUIEventAdapter& ea)
{
	osgUtil::LineSegmentIntersector::Intersections intersections;
	osgUtil::LineSegmentIntersector::Intersections::iterator itr;
		    
	float dX = lastX - ea.getXnormalized();
	float dY = lastY - ea.getYnormalized();

	float dXclick = clickX - ea.getXnormalized();
	float dYclick = clickY - ea.getYnormalized();

	unsigned int buttonMask = ea.getButtonMask();
	unsigned int modkeyMask = ea.getModKeyMask();

	
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
	
	
	/*
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
	*/
	
	
	if (this->picker)
	{


		
		// if a node is currently selected, then we look for a MOVE, RELEASE or
		// DRAG event
		if (selectedNode!=gensym("NULL"))
		{
			switch(ea.getEventType())
			{
				case(GUIEventAdapter::MOVE):
	    			sendEvent(selectedNode->s_name,
	    					"sisff",
	    					"event",
	    					(int)ea.getEventType(),
	    					user->id->s_name,
	    					dX,
	    					dY,
	    					LO_ARGS_END);
	    			break;	
	    			
	    		case(GUIEventAdapter::DRAG):
	    			sendEvent(selectedNode->s_name,
	    					"sisff",
	    					"event",
	    					(int)ea.getEventType(),
	    					user->id->s_name,
	    					dX,
	    					dY,
	    					LO_ARGS_END);
	    			break;	
	    		case(GUIEventAdapter::RELEASE):
	    			sendEvent(selectedNode->s_name,
			    			"sisff",
			    			"event",
			    			(int)ea.getEventType(),
			    			user->id->s_name,
			    			(float) ea.getModKeyMask(),
			    			(float) ea.getButtonMask(),
			    			LO_ARGS_END);
	    			selectedNode = gensym("NULL");
	    			break;
			}
		}
		
		// otherwise, we check to see if the mouse is intersecting with anything
		// and set the selectedNode in the case of a PUSH, or just send other
		// events as necessary
		
		else if (view->computeIntersections(ea.getX(),ea.getY(),intersections))
	    {  
			osg::ref_ptr<GroupNode> testNode;
			osg::Vec3 v;

			
			
			for (itr = intersections.begin(); itr != intersections.end(); ++itr)
			{
				// intersections are ordered from nearest to furthest, so we
				// iterate through and act upon the first intersection that can
				// be cast as an interactive SPIN node
				for (int i=(*itr).nodePath.size()-1; i>=0; i--)
				{
					testNode = dynamic_cast<GroupNode*>((*itr).nodePath[i]);
					if (testNode.valid() && (testNode->getInteractionMode())) // this is a interactive!
					{
						//std::cout << ea.getEventType() << " on SPIN node: " << testNode->id->s_name << std::endl;
						

				    	switch(ea.getEventType())
				    	{
				    		// send MOVE events (eg, for drawing on nodes)
				    		case(GUIEventAdapter::MOVE):
				    			sendEvent(testNode->id->s_name,
				    					"sisff",
				    					"event",
				    					(int)ea.getEventType(),
				    					user->id->s_name,
				    					dX,
				    					dY,
				    					LO_ARGS_END);
				    			break;
				    		
				    		// DRAG will only occur if someone clicks elsewhere
				    		// and rolls onto this node, so we shouldn't send
				    		// anything, should we?
				    		case(GUIEventAdapter::DRAG):
				    			break;

				    		// Same goes for the case where someone rolls onto a
				    		// node with the mouse button down, and releases.
				    		// However, just to be safe, we'll also ensure that
				    		// there is no selected node anymore:
				    		case(GUIEventAdapter::RELEASE):
				    			
				    			selectedNode = gensym("NULL");
				    			break;
				    		
				    		// SCROLLING (with the mouse wheel) is important. It
				    		// could be used to scale for example.
				    		case(GUIEventAdapter::SCROLL):
				    			sendEvent(testNode->id->s_name,
				    					"sisff",
				    					"event",
				    					(int)ea.getEventType(),
				    					user->id->s_name,
				    					scrollX,
				    					scrollY,
				    					LO_ARGS_END);
				    			break;
				    		
				    		// Finally, in the case of a PUSH, we both send the
				    		// event, and set the selectedNode
				    		case(GUIEventAdapter::PUSH):
				    			sendEvent(testNode->id->s_name,
				    					"sisff",
				    					"event",
				    					(int)ea.getEventType(),
				    					user->id->s_name,
				    					(float) ea.getModKeyMask(),
				    					(float) ea.getButtonMask(),
				    					LO_ARGS_END);
				    			selectedNode = testNode->id;
				    			break;
				    	}
						
				    	//v = (*itr).getLocalIntersectPoint();
				    	//lo_message_add( msg, "fff", v.x(), v.y(), v.z() );
			    		//v = (*itr).getWorldIntersectPoint();
			    		//lo_message_add(msg, "fff", v.x(), v.y(), v.z());
    
				    	// stop searching up the nodePath
						break;

					} // end if interactive
				}

				// stop searching through the intersections
				if (testNode.valid()) break;
				
				/*
				std::cout << "Got intersection: " << std::endl;
				osg::Vec3 dbgVect;
				std::cout<<"  ratio "<<(*itr).ratio<<std::endl;
				dbgVect = (*itr).getLocalIntersectPoint();
				std::cout<<"  localPoint  ("<<dbgVect.x()<<","<<dbgVect.y()<<","<<dbgVect.z()<<")"<<std::endl;
				dbgVect = (*itr).getLocalIntersectNormal();
				std::cout<<"  localNormal ("<<dbgVect.x()<<","<<dbgVect.y()<<","<<dbgVect.z()<<")"<<std::endl;
				dbgVect = (*itr).getWorldIntersectPoint();
				std::cout<<"  worldPoint  ("<<dbgVect.x()<<","<<dbgVect.y()<<","<<dbgVect.z()<<")"<<std::endl;
				dbgVect = (*itr).getWorldIntersectNormal();
				std::cout<<"  worldNormal ("<<dbgVect.x()<<","<<dbgVect.y()<<","<<dbgVect.z()<<")"<<std::endl;
				*/
				
				
			} // end intersection iterator
		} // end intersections test
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
				
				
				sendEvent(user->id->s_name, "sfff", "setVelocity", 0.0, 0.0, 0.0, LO_ARGS_END);
				if (buttonMask == (GUIEventAdapter::LEFT_MOUSE_BUTTON+GUIEventAdapter::RIGHT_MOUSE_BUTTON))
					sendEvent(user->id->s_name, "sfff", "setSpin", 0.0, 0.0, 0.0, LO_ARGS_END);

				
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
				    	sendEvent(user->id->s_name, "sfff", "move", dX*movScalar, dY*movScalar, 0.0, LO_ARGS_END);
				    }
				    else if (buttonMask == GUIEventAdapter::RIGHT_MOUSE_BUTTON)
				    {
				    	// pan up/down & left/right:
				    	sendEvent(user->id->s_name, "sfff", "move", dX*movScalar, 0.0, dY*movScalar, LO_ARGS_END);
				    }
				    else if (buttonMask == (GUIEventAdapter::LEFT_MOUSE_BUTTON+GUIEventAdapter::RIGHT_MOUSE_BUTTON))
				    {
				    	// rotate mode:
				    	sendEvent(user->id->s_name, "sfff", "rotate", dY*rotScalar, 0.0, dX*rotScalar, LO_ARGS_END);
				    }
				}
			
			    else if (modkeyMask==GUIEventAdapter::MODKEY_LEFT_CTRL)
			    {
			    	int dXsign, dYsign;
			    	(dXclick<0) ? dXsign=-1 : dXsign=1;
			    	(dYclick<0) ? dYsign=-1 : dYsign=1;
			    	
			    	// drive mode:
				    if (buttonMask == GUIEventAdapter::LEFT_MOUSE_BUTTON)
				    {
				    	// drive forward/back & left/right:
			    		sendEvent(user->id->s_name, "sfff", "setVelocity", dXsign*pow(dXclick*movScalar,2), dYsign*pow(dYclick*movScalar,2), 0.0f, LO_ARGS_END);
			    		
				    }
				    else if (buttonMask == GUIEventAdapter::RIGHT_MOUSE_BUTTON)
				    {
				    	// drive up/down & left/right:
			    		sendEvent(user->id->s_name, "sfff", "setVelocity", dXsign*pow(dXclick*movScalar,2), 0.0f, dYsign*pow(dYclick*movScalar,2), LO_ARGS_END);
			    	}
				    else if (buttonMask == (GUIEventAdapter::LEFT_MOUSE_BUTTON+GUIEventAdapter::RIGHT_MOUSE_BUTTON))
				    {
				    	// rotate mode:
			    		sendEvent(user->id->s_name, "sfff", "setSpin", dYsign*pow(dYclick*rotScalar,2), 0.0, dXsign*pow(dXclick*rotScalar,2), LO_ARGS_END);
				    }
			    }

				break;
				
			case(GUIEventAdapter::RELEASE):
				//sendEvent(user->id->s_name, "sfff", "setVelocity", 0.0, 0.0, 0.0, LO_ARGS_END);
				//sendEvent(user->id->s_name, "sfff", "setSpin", 0.0, 0.0, 0.0, LO_ARGS_END);
				break;
				
			case(GUIEventAdapter::SCROLL):
				// zoom?
				break;
			default:
				// here we could pop up an HUD menu if someone double-clicks or right-clicks
				break;
		}
	}

	lastX = ea.getXnormalized();
	lastY = ea.getYnormalized();
}
