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

using namespace osgGA;

ViewerManipulator::ViewerManipulator(spinContext* s, UserNode *u)
{
	this->spin = s;
	this->user = u;
	this->picker = false;
}

ViewerManipulator::~ViewerManipulator() {}

void ViewerManipulator::enablePicking(bool b)
{
	this->picker = b;
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
		spin->sendNodeMessage(user->id->s_name, "sfff", "setOrientation", 0.0, 0.0, 0.0, LO_ARGS_END);
	}
}

void ViewerManipulator::processEvent(osgViewer::View* view, const GUIEventAdapter& ea)
{

	/*
	float dX = (lastX - ea.getX()) / ea.getWindowWidth();
	float dY = (lastY - ea.getY()) / ea.getWindowHeight();
	*/
	
	float dX = lastX - ea.getXnormalized();
	float dY = lastY - ea.getYnormalized();
	
	float dXclick = clickX - ea.getXnormalized();
	float dYclick = clickY - ea.getYnormalized();

	
	unsigned int buttonMask = ea.getButtonMask();
	unsigned int modkeyMask = ea.getModKeyMask();
	
	// a boolean that tells us if we should move the camera as well (will become
	// false if there is an pick event)
	bool processSceneEvent = true;
	
	
	
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
	*/
	
	
	if (this->picker)
	{
		
	
	    osgUtil::LineSegmentIntersector::Intersections intersections;
	    osgUtil::LineSegmentIntersector::Intersections::iterator itr;
	    if (view->computeIntersections(ea.getX(),ea.getY(),intersections))
	    {  
			osg::ref_ptr<GroupNode> testNode;
			lo_message msg;
			osg::Vec3 v;
			
			
			for (itr = intersections.begin(); itr != intersections.end(); ++itr)
			{
				for (int i=(*itr).nodePath.size()-1; i>=0; i--)
				{
					testNode = dynamic_cast<GroupNode*>((*itr).nodePath[i]);
					if (testNode.valid() && (testNode->getInteractionMode()==GroupNode::DRAGGABLE)) // this is a Draggable!
					{
						// only add the hit if not already in the list (note that
						// one node might have several intersections, for each face
						// that instersects with the line segment)
						//vector<t_symbol*>::iterator found = std::find( (*itr).begin(), (*itr).end(), testNode->id );
						//if ( found == (*itr).end() )
						{
							//std::cout << ea.getEventType() << " on SPIN node: " << testNode->id->s_name << std::endl;
							processSceneEvent = false;
							
				    		msg = lo_message_new();
				    		
				    		// first add the event type, and user name:
				    		lo_message_add( msg, "sis", "pickEvent", (int)ea.getEventType(), user->id->s_name);
							
				    		// add the 2 data floats for the event:
					    	switch(ea.getEventType())
					    	{
					    		case(GUIEventAdapter::MOVE):
					    			lo_message_add( msg, "ff", dX, dY );
					    			break;
					    		case(GUIEventAdapter::DRAG):
					    			lo_message_add( msg, "ff", dX, dY );
					    			break;
					    		case(GUIEventAdapter::SCROLL):
					    			lo_message_add( msg, "ff", ea.getScrollingDeltaX(), ea.getScrollingDeltaY() );
					    			break;
					    		default:
					    			lo_message_add( msg, "ff", (float) ea.getModKeyMask(), (float) ea.getButtonMask() );
					    			break;
					    	}
							
					    	// finally, add the intersection point:
					    	//v = (*itr).getLocalIntersectPoint();
					    	//lo_message_add( msg, "fff", v.x(), v.y(), v.z() );
				    		
				    		//v = (*itr).getWorldIntersectPoint();
				    		//lo_message_add(msg, "fff", v.x(), v.y(), v.z());
			
					    	spin->sendNodeMessage(testNode->id->s_name, msg);
							
						}
						break;
					}
				}
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
			}
		}
	}
	
    
    // scene event processing (eg, camera motion):
	if (processSceneEvent)
	{
		float movScalar = 10.0;
		float rotScalar = 20.0;
		
		switch(ea.getEventType())
		{
			case(GUIEventAdapter::PUSH):
				// in the case of a mouse click, we store the current coords
				clickX = ea.getXnormalized();
				clickY = ea.getYnormalized();
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
				    	spin->sendNodeMessage(user->id->s_name, "sfff", "move", dX*movScalar, dY*movScalar, 0.0, LO_ARGS_END);
				    }
				    else if (buttonMask == GUIEventAdapter::RIGHT_MOUSE_BUTTON)
				    {
				    	// pan up/down & left/right:
				    	spin->sendNodeMessage(user->id->s_name, "sfff", "move", dX*movScalar, 0.0, dY*movScalar, LO_ARGS_END);
				    }
				    else if (buttonMask == (GUIEventAdapter::LEFT_MOUSE_BUTTON+GUIEventAdapter::RIGHT_MOUSE_BUTTON))
				    {
				    	// rotate mode:
				    	spin->sendNodeMessage(user->id->s_name, "sfff", "rotate", dY*rotScalar, 0.0, dX*rotScalar, LO_ARGS_END);
				    }
				}
			
			    else if (modkeyMask==GUIEventAdapter::MODKEY_LEFT_CTRL)
			    {
			    	// drive mode:
				    if (buttonMask == GUIEventAdapter::LEFT_MOUSE_BUTTON)
				    {
				    	// drive forward/back & left/right:
				    	
				    	std::cout << "dClick: " << dXclick<<","<<dYclick << std::endl;
				    	//spin->sendNodeMessage(user->id->s_name, "sfff", "setVelocity", pow(2,dXclick*movScalar)-1, pow(2,dYclick*movScalar)-1, 0.0f, LO_ARGS_END);
				    	
			    		spin->sendNodeMessage(user->id->s_name, "sfff", "setVelocity", dXclick*movScalar*2, dYclick*movScalar*2, 0.0f, LO_ARGS_END);
			    		
				    }
				    else if (buttonMask == GUIEventAdapter::RIGHT_MOUSE_BUTTON)
				    {
				    	// drive up/down & left/right:
			    		spin->sendNodeMessage(user->id->s_name, "sfff", "setVelocity", dXclick*movScalar*2, 0.0f, dYclick*movScalar*2, LO_ARGS_END);
			    	}
				    else if (buttonMask == (GUIEventAdapter::LEFT_MOUSE_BUTTON+GUIEventAdapter::RIGHT_MOUSE_BUTTON))
				    {
				    	// rotate mode:
			    		spin->sendNodeMessage(user->id->s_name, "sfff", "setSpin", dYclick*rotScalar, 0.0, dXclick*rotScalar, LO_ARGS_END);
				    }
			    }

				break;
				
			case(GUIEventAdapter::RELEASE):
				spin->sendNodeMessage(user->id->s_name, "sfff", "setVelocity", 0.0, 0.0, 0.0, LO_ARGS_END);
				spin->sendNodeMessage(user->id->s_name, "sfff", "setSpin", 0.0, 0.0, 0.0, LO_ARGS_END);
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
