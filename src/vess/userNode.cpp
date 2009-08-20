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
//  You should have received a copy of the Lesser GNU General Public License
//  along with SPIN Framework. If not, see <http://www.gnu.org/licenses/>.
// -----------------------------------------------------------------------------

#include "userNode.h"
#include "asSceneManager.h"
#include "osgUtil.h"

using namespace std;


// ===================================================================
// constructor:
userNode::userNode (asSceneManager *sceneManager, char *initID) : asBasicNode(sceneManager, initID)
{
	nodeType = "userNode";

	name = string(initID);
	host = "localhost";
	
	setReportMode(asBasicNode::GLOBAL_6DOF);

}

// ===================================================================
// destructor
userNode::~userNode()
{

}



// ===================================================================
// ======================== SET METHODS: =============================
// ===================================================================

void userNode::setName (const char *newvalue)
{
	name = string(newvalue);
	BROADCAST(this, "ss", "setName", getName());
}

void userNode::setHost (const char *newvalue)
{
	host = string(newvalue);
	BROADCAST(this, "ss", "setHost", getHost());
}


std::vector<lo_message> userNode::getState ()
{
	// inherit state from base class
	std::vector<lo_message> ret = asBasicNode::getState();

	lo_message msg;

	msg = lo_message_new();
	lo_message_add(msg, "ss", "setName", getName());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "ss", "setHost", getHost());
	ret.push_back(msg);

	return ret;
}
