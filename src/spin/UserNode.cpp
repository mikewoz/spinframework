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

#include "UserNode.h"
#include "SceneManager.h"
#include "spinApp.h"
#include "spinBaseContext.h"

using namespace std;


// *****************************************************************************
// constructor:
UserNode::UserNode (SceneManager *sceneManager, char *initID) : ConstraintsNode(sceneManager, initID)
{
    nodeType = "UserNode";
    this->setName(string(id->s_name) + ".UserNode");

    description_ = string(initID);

    setTranslation(0.0, -5.0, 0.5);
    setReportMode(GroupNode::GLOBAL_6DOF);

    ping();
}

// destructor
UserNode::~UserNode()
{
    //std::cout << "Destroying UserNode: " << id->s_name << std::endl;
}

// *****************************************************************************

void UserNode::callbackUpdate()
{
	ConstraintsNode::callbackUpdate();


	osg::Timer_t t = osg::Timer::instance()->tick();
	if (osg::Timer::instance()->delta_s(lastPing_,t) > 10) // every 60 seconds
	{
		// this user stopped pinging, so we should remove him from the
		// subgraph.

		//uncomment when ready:
		this->scheduleForDeletion = true;
	}


}

void UserNode::updateNodePath()
{
    GroupNode::updateNodePath();
    this->nodepathUpdate = true;
}

// *****************************************************************************

void UserNode::setDescription (const char *newvalue)
{
    description_ = string(newvalue);
    BROADCAST(this, "ss", "setDescription", getDescription());
}

void UserNode::ping()
{
	lastPing_ = osg::Timer::instance()->tick();
}

// *****************************************************************************

std::vector<lo_message> UserNode::getState ()
{
	// inherit state from base class
	std::vector<lo_message> ret = ConstraintsNode::getState();

	lo_message msg;

	msg = lo_message_new();
	lo_message_add(msg, "ss", "setDescription", getDescription());
	ret.push_back(msg);


	return ret;
}
