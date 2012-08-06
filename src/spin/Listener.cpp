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

#include "Listener.h"
#include "SceneManager.h"
#include "spinApp.h"
#include "spinBaseContext.h"

#ifdef WITH_SPATOSC
#include <spatosc/spatosc.h>
#endif

namespace spin
{

// ===================================================================
// constructor:
Listener::Listener (SceneManager *sceneManager, const char* initID) : DSPNode(sceneManager, initID)
{
	this->setNodeType("Listener");
	this->setName(this->getID() + ".Listener");

#ifdef WITH_SPATOSC
	if (spinApp::Instance().hasAudioRenderer)
	{
        spatOSCListener = spinApp::Instance().audioScene->createListener(this->getID());
        std::cout << "Created SpatOSC Listener:" << std::endl;
        //spinApp::Instance().audioScene->debugPrint();
	}
#endif

    // after the spatosc node is created, set some params:
	//this->setURI("plugin://listener-stereo~");

}

// ===================================================================
// destructor
Listener::~Listener()
{
#ifdef WITH_SPATOSC
	if (spinApp::Instance().hasAudioRenderer)
	{
	    spinApp::Instance().audioScene->deleteNode(spatOSCListener);
        std::cout << "Deleted SpatOSC Listener:" << std::endl;
        //spinApp::Instance().audioScene->debugPrint();
	}
#endif
    
}
// ===================================================================
void Listener::callbackUpdate(osg::NodeVisitor* nv)
{
    // need to first call the superclass update method (specifically, GroupNode)
    // which will update globalMatrix_
    DSPNode::callbackUpdate(nv);

    // now, we can get the global position and orientation, and we can forward
    // it to SpatOSC

#ifdef WITH_SPATOSC
    if (spinApp::Instance().hasAudioRenderer)
    {
        this->globalMatrix_ = getGlobalMatrix(); // in case reporting is off
        osg::Vec3 myPos = globalMatrix_.getTrans();
        osg::Vec3 myRot = QuatToEuler(globalMatrix_.getRotate());

        spatOSCListener->setPosition(myPos.x(), myPos.y(), myPos.z());
        spatOSCListener->setOrientation(myRot.x(), myRot.y(), myRot.z());
    }
#endif
}

void Listener::setParam (const char *paramName, const char *paramValue)
{
    ReferencedNode::setParam(paramName,paramValue);
#ifdef WITH_SPATOSC
    if (spinApp::Instance().hasAudioRenderer)
    {
        spatOSCListener->setStringProperty(paramName, paramValue);
    }
#endif
}

void Listener::setParam (const char *paramName, float paramValue)
{
    ReferencedNode::setParam(paramName,paramValue);
#ifdef WITH_SPATOSC
    if (spinApp::Instance().hasAudioRenderer)
    {
        spatOSCListener->setFloatProperty(paramName, paramValue);
    }
#endif
}

void Listener::setURI (const char *uri)
{
    DSPNode::setURI(uri);
#ifdef WITH_SPATOSC
    if (spinApp::Instance().hasAudioRenderer)
    {
        spatOSCListener->setURI(this->getURI());
    }
#endif
}

std::vector<lo_message> Listener::getState () const
{
	// inherit state from base class
	std::vector<lo_message> ret = DSPNode::getState();
	
	
	return ret;
}

} // end of namespace spin

