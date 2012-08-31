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

#include <iostream>
#include <osg/Fog>

#include "Fog.h"
#include "spinApp.h" // for BROADCAST
#include "spinBaseContext.h"
#include "SceneManager.h"

namespace spin
{

// *****************************************************************************
// constructor:
Fog::Fog (SceneManager *s, const char *initID) : ReferencedStateSet(s, initID)
{
	classType_ = "Fog";
 	this->setName(this->getID() + ".Fog");

	fog_ = new osg::Fog();

	fog_->setMode(osg::Fog::EXP);
    fog_->setStart(-100.f);
    fog_->setDensity(0.005);

	setFogColor(1.0, 1.0, 1.0, 0.0);
	
	
	osg::StateSet *worldStateSet = sceneManager_->worldNode->getOrCreateStateSet();
    worldStateSet->setMode(GL_FOG, osg::StateAttribute::ON);
    worldStateSet->setAttribute(fog_, osg::StateAttribute::ON);
}

// destructor
Fog::~Fog()
{
	osg::StateSet *worldStateSet = sceneManager_->worldNode->getOrCreateStateSet();
    worldStateSet->setMode(GL_FOG, osg::StateAttribute::OFF);
}

// *****************************************************************************
void Fog::setFogDensity (float density)
{
	fog_->setDensity(density);
	BROADCAST(this, "sf", "setFogDensity", getFogDensity());
}

float Fog::getFogDensity() const { return fog_->getDensity(); }

void Fog::setFogColor (float r, float g, float b, float a)
{
	osg::Vec4 newColor = osg::Vec4(r, g, b, a);
	fog_->setColor(newColor);
	sceneManager_->rootNode->setClearColor(newColor);
	BROADCAST(this, "sffff", "setFogColor", r, g, b, a);
}

osg::Vec4 Fog::getFogColor() const { return fog_->getColor(); }

// *****************************************************************************
std::vector<lo_message> Fog::getState () const
{
	// inherit state from base class
	std::vector<lo_message> ret = ReferencedStateSet::getState();
		
	lo_message msg;
	
	msg = lo_message_new();
	lo_message_add(msg, "sf", "setFogDensity", getFogDensity());
	ret.push_back(msg);
	
	msg = lo_message_new();
	osg::Vec4 v4 = this->getFogColor();
	lo_message_add(msg, "sffff", "setFogColor", v4.x(), v4.y(), v4.z(), v4.w());
	ret.push_back(msg);
	
	return ret;
}

} // end of namespace spin
