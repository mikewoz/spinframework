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

using namespace std;


// ===================================================================
// constructor:
Listener::Listener (SceneManager *sceneManager, char *initID) : SoundNode(sceneManager, initID)
{
	nodeType = "Listener";
	this->setPlugin("listener-stereo~");
	type = "listener-stereo.connn~";
	this->setName(string(id->s_name) + ".Listener");
}

// ===================================================================
// destructor
Listener::~Listener()
{
}


void Listener::setType (const char* t)
{
	// only do this if the type has changed:
	if (type == std::string(t)) return;
	type = std::string(t);
	
    BROADCAST(this, "ss", "setType", getType());
    
}

std::vector<lo_message> Listener::getState ()
{
	// inherit state from base class
	std::vector<lo_message> ret = SoundNode::getState();
	
	lo_message msg = lo_message_new();
	lo_message_add(msg,  "ss", "setType", getType());
	ret.push_back(msg);
	
	return ret;
}