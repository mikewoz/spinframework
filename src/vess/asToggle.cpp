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
//    La SociŽtŽ des Arts Technologiques (http://www.sat.qc.ca)
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
//  You should have received a copy of the Lesser GNU General Public License
//  along with SPIN Framework. If not, see <http://www.gnu.org/licenses/>.
// -----------------------------------------------------------------------------

#include "asToggle.h"
#include "asSceneManager.h"

using namespace std;

//extern asSceneManager *sceneManager;


// *****************************************************************************
// constructor:
asToggle::asToggle (asSceneManager *sceneManager, char *initID) : asReferenced(sceneManager, initID)
{
	this->setName(string(id->s_name) + ".asToggle");
	nodeType = "asToggle";

	this->setNodeMask(STATSDATA_NODE_MASK); // nodemask info in asGlobals.h
	
	toggle = false;
}

// *****************************************************************************
// destructor
asToggle::~asToggle()
{

}


// *****************************************************************************
void asToggle::setToggle (int b)
{
	if (this->toggle != (bool)b)
	{
		this->toggle = (bool) b;
		BROADCAST(this, "si", "setToggle", (int) this->toggle);	
	}
}

// *****************************************************************************

std::vector<lo_message> asToggle::getState ()
{
	// inherit state from base class
	std::vector<lo_message> ret = asReferenced::getState();
	
	lo_message msg;
	
	msg = lo_message_new();
	lo_message_add(msg, "si", "setToggle", (int)toggle);
	ret.push_back(msg);
	
	return ret;
}