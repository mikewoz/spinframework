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

#include "spinApp.h"
#include "SoundNode.h"
#include "SceneManager.h"
#ifdef WITH_SPATOSC
#include <spatosc/spatosc.h>
#endif

namespace spin
{

// ===================================================================
// constructor:
SoundNode::SoundNode (SceneManager *sceneManager, char *initID) : DSPNode(sceneManager, initID)
{
	nodeType = "SoundNode";
	this->setName(std::string(id->s_name) + ".SoundNode");

#ifdef WITH_SPATOSC
	if (spinApp::Instance().hasAudioRenderer)
	{
	    spatOSCSource = spinApp::Instance().audioScene->createSoundSource(std::string(id->s_name));
        std::cout << "Created SpatOSC Source:" << std::endl;
        spinApp::Instance().audioScene->debugPrint();
	}
#endif
}

// destructor
SoundNode::~SoundNode()
{
	//std::cout << "In SoundNode destructor... node: " << this->id->s_name << std::endl;
#ifdef WITH_SPATOSC
	if (spinApp::Instance().hasAudioRenderer)
	{
	    spinApp::Instance().audioScene->deleteNode(spatOSCSource);
        std::cout << "Deleted SpatOSC Source:" << std::endl;
        spinApp::Instance().audioScene->debugPrint();
	}
#endif
    
}

// ===================================================================
void SoundNode::callbackUpdate()
{
    // need to first call the superclass update method (specifically, GroupNode)
    // which will update _globalMatrix (since reporting is on)
    DSPNode::callbackUpdate();

    // now, we can get the global position and orientation, and we can forward
    // it to SpatOSC

#ifdef WITH_SPATOSC
    if (spinApp::Instance().hasAudioRenderer)
    {
        osg::Vec3 myPos = _globalMatrix.getTrans();
        osg::Vec3 myRot = QuatToEuler(_globalMatrix.getRotate());

        spatOSCSource->setPosition(myPos.x(), myPos.y(), myPos.z());
        spatOSCSource->setOrientation(myRot.x(), myRot.y(), myRot.z());
    }
#endif
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
    {
        this->_globalMatrix = getGlobalMatrix();
        osg::Vec3 myPos = _globalMatrix.getTrans();
        spatOSCSource->setPosition(myPos.x(), myPos.y(), myPos.z());
    }
#endif

}

void SoundNode::setOrientation (float p, float r, float y)
{
    GroupNode::setOrientation(p,r,y);
#ifdef WITH_SPATOSC
    if (spinApp::Instance().hasAudioRenderer)
    {
        this->_globalMatrix = getGlobalMatrix();
        osg::Vec3 myRot = QuatToEuler(_globalMatrix.getRotate());
        spatOSCSource->setOrientation(myRot.x(), myRot.y(), myRot.z());
    }
#endif
}

void SoundNode::setOrientationQuat (float x, float y, float z, float w)
{
    GroupNode::setOrientationQuat(x,y,z,w);
#ifdef WITH_SPATOSC
    if (spinApp::Instance().hasAudioRenderer)
    {
        this->_globalMatrix = getGlobalMatrix();
        osg::Vec3 myRot = QuatToEuler(_globalMatrix.getRotate());
        spatOSCSource->setOrientation(myRot.x(), myRot.y(), myRot.z());
    }
#endif
}

std::vector<lo_message> SoundNode::getState () const
{
	// inherit state from base class
	std::vector<lo_message> ret = DSPNode::getState();
	
	//lo_message msg;
	

	
	
	return ret;
}

} // end of namespace spin

