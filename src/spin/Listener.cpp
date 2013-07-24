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

#include "listener.h"
#include "scenemanager.h"
#include "spinapp.h"
#include "spinbasecontext.h"

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
void Listener::debug()
{
    DSPNode::debug();
    
#ifdef WITH_SPATOSC
    if (spinApp::Instance().hasAudioRenderer)
    {
        std::cout << "-------------" << std::endl;
        std::cout << "SpatOSC data:" << std::endl;
        spatOSCListener->debugPrint();
    }
#endif
    
}

void Listener::callbackUpdate(osg::NodeVisitor* nv)
{
    // need to first call the superclass update method (specifically, GroupNode)
    // which will update globalMatrix_
    DSPNode::callbackUpdate(nv);
}

bool Listener::dumpGlobals(bool forced)
{
    DSPNode::dumpGlobals(forced);
#ifdef WITH_SPATOSC
    if (spinApp::Instance().hasAudioRenderer)
    {
        this->globalMatrix_ = getGlobalMatrix(); // in case reporting is off
        osg::Vec3 myPos = globalMatrix_.getTrans();
        osg::Vec3 myRot = Vec3inDegrees(QuatToEuler(globalMatrix_.getRotate()));

        spatOSCListener->setPosition(myPos.x(), myPos.y(), myPos.z());
        spatOSCListener->setOrientation(myRot.x(), myRot.y(), myRot.z());
    }
#endif
    return 1;
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

void Listener::setTranslation (float x, float y, float z)
{
    GroupNode::setTranslation(x,y,z);
#ifdef WITH_SPATOSC
    if (spinApp::Instance().hasAudioRenderer)
        this->dumpGlobals(true);
#endif

}

void Listener::setOrientation (float p, float r, float y)
{
    GroupNode::setOrientation(p,r,y);
#ifdef WITH_SPATOSC
    if (spinApp::Instance().hasAudioRenderer)
        this->dumpGlobals(true);
#endif
}

void Listener::setOrientationQuat (float x, float y, float z, float w)
{
    GroupNode::setOrientationQuat(x,y,z,w);
#ifdef WITH_SPATOSC
    if (spinApp::Instance().hasAudioRenderer)
        this->dumpGlobals(true);
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

void Listener::sendEvent (const char *types, lo_arg **argv, int argc )
{
#ifdef WITH_SPATOSC
    if (spinApp::Instance().hasAudioRenderer)
    {
        std::string node = this->getID();
        std::string key;
        std::vector<std::string> value;
        
        // first item should be the key string, followd by any corresponing items
        if (! lo_is_numerical_type((lo_type)types[0]))
        {
            key = (const char*) argv[0];
            
        }
        else
        {
            std::cerr << "ERROR: Listener::sendEvent 1st item must be a key <string> " <<  std::endl;
            return;
        }
        
        for (int i = 1; i < argc; i++)
        {
            if (lo_is_numerical_type((lo_type) types[i]))
            {
                std::ostringstream os;
                os << (float) lo_hires_val((lo_type) types[i], argv[i]);
                value.push_back(os.str());
            }
            else
            {
                value.push_back((const char*) argv[i] );
            }
        }
        // std::cout << "Listener::sendEvent CALLED for node:  " << this->getID() <<  "with key: " << key << std::endl;
        spatOSCListener->sendEvent(key, value);
    }
#endif
}

    
} // end of namespace spin

