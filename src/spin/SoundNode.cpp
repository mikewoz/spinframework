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

#include "spinapp.h"
#include "soundnode.h"
#include "scenemanager.h"
#ifdef WITH_SPATOSC
#include <spatosc/spatosc.h>
#endif

namespace spin
{

// ===================================================================
// constructor:
SoundNode::SoundNode (SceneManager *sceneManager, const char* initID) : DSPNode(sceneManager, initID)
{
	this->setNodeType("SoundNode");
	this->setName(this->getID() + ".SoundNode");

#ifdef WITH_SPATOSC
	if (spinApp::Instance().hasAudioRenderer)
	{
	    spatOSCSource = spinApp::Instance().audioScene->createSoundSource(this->getID());
        std::cout << "Created SpatOSC Source:" << std::endl;
        //spinApp::Instance().audioScene->debugPrint();
	}
#endif

    // after the spatosc node is created, set some params:
    
}

// destructor
SoundNode::~SoundNode()
{
	//std::cout << "In SoundNode destructor... node: " << this->getID() << std::endl;
#ifdef WITH_SPATOSC
	if (spinApp::Instance().hasAudioRenderer)
	{
	    spinApp::Instance().audioScene->deleteNode(spatOSCSource);
        std::cout << "Deleted SpatOSC Source:" << std::endl;
        //spinApp::Instance().audioScene->debugPrint();
	}
#endif
    
}

// ===================================================================

void SoundNode::debug()
{
    DSPNode::debug();
    
#ifdef WITH_SPATOSC
    if (spinApp::Instance().hasAudioRenderer)
    {
    std::cout << "-------------" << std::endl;
    std::cout << "SpatOSC data:" << std::endl;
        spatOSCSource->debugPrint();
    }
#endif
    
}

void SoundNode::callbackUpdate(osg::NodeVisitor* nv)
{
    // need to first call the superclass update method (specifically, GroupNode)
    // which will update _globalMatrix (since reporting is on)
    DSPNode::callbackUpdate(nv);
}

bool SoundNode::dumpGlobals(bool forced)
{
    DSPNode::dumpGlobals(forced);
    
#ifdef WITH_SPATOSC
    if (spinApp::Instance().hasAudioRenderer)
    {
        this->globalMatrix_ = getGlobalMatrix(); // in case reporting is off
        osg::Vec3 myPos = globalMatrix_.getTrans();
        osg::Vec3 myRot = Vec3inDegrees(QuatToEuler(globalMatrix_.getRotate()));

        spatOSCSource->setPosition(myPos.x(), myPos.y(), myPos.z());
        spatOSCSource->setOrientation(myRot.x(), myRot.y(), myRot.z());
    }
#endif
    return 1;
}

void SoundNode::setParam (const char *paramName, const char *paramValue)
{
    ReferencedNode::setParam(paramName,paramValue);
#ifdef WITH_SPATOSC
    if (spinApp::Instance().hasAudioRenderer)
    {
        spatOSCSource->setStringProperty(paramName, paramValue);
    }
#endif
}

void SoundNode::setParam (const char *paramName, float paramValue)
{
    ReferencedNode::setParam(paramName,paramValue);
#ifdef WITH_SPATOSC
    if (spinApp::Instance().hasAudioRenderer)
    {
        spatOSCSource->setFloatProperty(paramName, paramValue);
    }
#endif
}

void SoundNode::setTranslation (float x, float y, float z)
{
    GroupNode::setTranslation(x,y,z);
#ifdef WITH_SPATOSC
    if (spinApp::Instance().hasAudioRenderer)
        this->dumpGlobals(true);
#endif

}

void SoundNode::setOrientation (float p, float r, float y)
{
    GroupNode::setOrientation(p,r,y);
#ifdef WITH_SPATOSC
    if (spinApp::Instance().hasAudioRenderer)
        this->dumpGlobals(true);
#endif
}

void SoundNode::setOrientationQuat (float x, float y, float z, float w)
{
    GroupNode::setOrientationQuat(x,y,z,w);
#ifdef WITH_SPATOSC
    if (spinApp::Instance().hasAudioRenderer)
        this->dumpGlobals(true);
#endif
}

void SoundNode::setActive (int b)
{
    DSPNode::setActive(b);
#ifdef WITH_SPATOSC
    if (spinApp::Instance().hasAudioRenderer)
    {
        printf("setActive %f", b);
        spatOSCSource->setActive((bool)this->getActive());
    }
#endif
}

void SoundNode::setRadius (float f)
{
    DSPNode::setRadius(f);
#ifdef WITH_SPATOSC
    if (spinApp::Instance().hasAudioRenderer)
    {
        printf("setRadius %f", f);
        spatOSCSource->setRadius(this->getRadius());
    }
#endif
}

void SoundNode::setTransitionFactor (float f)
{
#ifdef WITH_SPATOSC
    if (spinApp::Instance().hasAudioRenderer)
    {
        spatOSCSource->setTransitionRadiusFactor(f);
    }
#endif
}


void SoundNode::setURI (const char *uri)
{
    DSPNode::setURI(uri);
#ifdef WITH_SPATOSC
    if (spinApp::Instance().hasAudioRenderer)
    {
        spatOSCSource->setURI(this->getURI());
    }
#endif
}

void SoundNode::setDirectivity(const char* horizPattern, const char* vertPattern)
{
#ifdef WITH_SPATOSC
    if (spinApp::Instance().hasAudioRenderer)
    {
        spinApp::Instance().audioScene->setDirectivity(spatOSCSource, horizPattern, vertPattern);
    }
#endif
}

void SoundNode::connect (const char* sinkNodeID)
{
#ifdef WITH_SPATOSC
    if (spinApp::Instance().hasAudioRenderer)
    {
        spatosc::Listener *listener = spinApp::Instance().audioScene->getListener(sinkNodeID);
        if (!listener)
        {
            std::cout << "ERROR: SoundNode::connect could not find sink: '" << sinkNodeID << "'" << std::endl;
            return;
        }
        spinApp::Instance().audioScene->connect(spatOSCSource, listener);
    }
#endif
}

void SoundNode::disconnect (const char* sinkNodeID)
{
#ifdef WITH_SPATOSC
    if (spinApp::Instance().hasAudioRenderer)
    {
        spatosc::Listener *listener = spinApp::Instance().audioScene->getListener(sinkNodeID);
        if (!listener)
        {
            std::cout << "ERROR: SoundNode::disconnect could not find sink: '" << sinkNodeID << "'" << std::endl;
            return;
        }
        spinApp::Instance().audioScene->disconnect(spatOSCSource, listener);
    }
#endif
}

void SoundNode::setConnectionParam (const char* sinkNodeID, const char* method, float value)
{
#ifdef WITH_SPATOSC
    if (spinApp::Instance().hasAudioRenderer)
    {
        spatosc::Listener *listener = spinApp::Instance().audioScene->getListener(sinkNodeID);
        if (!listener)
        {
            std::cout << "ERROR: SoundNode::setConnectionParam could not find sink: '" << sinkNodeID << "'" << std::endl;
            return;
        }
        spatosc::Connection* conn = spinApp::Instance().audioScene->getConnection(spatOSCSource, listener);
        if (!conn)
        {
            std::cout << "ERROR: SoundNode::setConnectionParam could not find connection between '" << getID() << "' and '" << sinkNodeID << "'" << std::endl;
            return;        
        }
        
        if (std::string(method)=="setDistanceFactor")
        {
            conn->setDistanceFactor(value);
        }
        else if (std::string(method)=="setDopplerFactor")
        {
            conn->setDopplerFactor(value);
        }
        else if (std::string(method)=="setRolloffFactor")
        {
            conn->setRolloffFactor(value);
        }
        else if (std::string(method)=="setMaxGainClip")
        {
            conn->setMaxGainClip(value);
        }
        else if (std::string(method)=="setConnectionMute")
        {
            if ((bool)value)
                conn->mute();
            else
                conn->unmute();
        }
    }
#endif
}

void SoundNode::sendEvent (const char *types, lo_arg **argv, int argc )
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
            std::cerr << "ERROR: SoundNode::sendEvent 1st item must be a key <string> " <<  std::endl;
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
        // std::cout << "SoundNode::sendEvent CALLED for node:  " << this->getID() <<  "with key: " << key << std::endl;
        spatOSCSource->sendEvent(key, value);
        
    // for( std::vector<std::string>::const_iterator i = value.begin(); i != value.end(); ++i)
    //   std::cout << *i << ' ' << std::endl;

    }
#endif
}


std::vector<lo_message> SoundNode::getState () const
{
	// inherit state from base class
	std::vector<lo_message> ret = DSPNode::getState();
	
#ifdef WITH_SPATOSC	
    if (spinApp::Instance().hasAudioRenderer)
    {    
        lo_message msg;
        
        msg = lo_message_new();
        lo_message_add(msg, "sss", "setDirectivity", spatOSCSource->getLateralDirectivity().c_str(), spatOSCSource->getVerticalDirectivity().c_str());
        ret.push_back(msg);
        
        msg = lo_message_new();
        lo_message_add(msg, "sf", "setTransitionFactor", spatOSCSource->getTransitionFactor());
        ret.push_back(msg);

	}
#endif
	
	return ret;
}

} // end of namespace spin

