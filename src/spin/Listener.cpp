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

using namespace std;

namespace spin
{

// ===================================================================
// constructor:
Listener::Listener (SceneManager *sceneManager, char *initID) : DSPNode(sceneManager, initID)
{
	nodeType = "Listener";
	this->setPlugin("listener-stereo~");
	type = "listener-stereo.conn~";
	this->setName(string(id->s_name) + ".Listener");

#ifdef WITH_SPATOSC
	if (spinApp::Instance().hasAudioRenderer)
	{
        spatOSCListener = spinApp::Instance().audioScene->createListener(std::string(id->s_name));
        std::cout << "Created SpatOSC Listener:" << std::endl;
        //spinApp::Instance().audioScene->debugPrint();
	}
#endif
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
void Listener::callbackUpdate()
{
    // need to first call the superclass update method (specifically, GroupNode)
    // which will update globalMatrix_
    DSPNode::callbackUpdate();

    // now, we can get the global position and orientation, and we can forward
    // it to SpatOSC

#ifdef WITH_SPATOSC
    if (spinApp::Instance().hasAudioRenderer)
    {
        osg::Vec3 myPos = globalMatrix_.getTrans();
        osg::Vec3 myRot = QuatToEuler(globalMatrix_.getRotate());

        spatOSCListener->setPosition(myPos.x(), myPos.y(), myPos.z());
        spatOSCListener->setOrientation(myRot.x(), myRot.y(), myRot.z());
    }
#endif
}

void Listener::setType (const char* t)
{
	// only do this if the type has changed:
	if (type == std::string(t)) return;
	type = std::string(t);
	
    BROADCAST(this, "ss", "setType", getType());
}

std::vector<lo_message> Listener::getState () const
{
	// inherit state from base class
	std::vector<lo_message> ret = DSPNode::getState();
	
	lo_message msg = lo_message_new();
	lo_message_add(msg,  "ss", "setType", getType());
	ret.push_back(msg);
	
	return ret;
}

} // end of namespace spin

