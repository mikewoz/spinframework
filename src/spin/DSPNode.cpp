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
#include "DSPNode.h"
#include "SoundConnection.h"

#include "SceneManager.h"
#include "spinApp.h"
#include "spinBaseContext.h"

//extern SceneManager *sceneManager;

// *****************************************************************************
// constructor:
DSPNode::DSPNode (SceneManager *sceneManager, char *initID) : GroupNode(sceneManager, initID)
{

	nodeType = "DSPNode";
	
	// enable report of globals by default:
	setReportMode(GroupNode::GLOBAL_6DOF);
	
	// connection stuff:
	connectTO.clear();
	connectFROM.clear();
	
	active = 1;
	
	plugin = "empty~";	
	
}

// *****************************************************************************
// destructor
DSPNode::~DSPNode()
{
	// we should check if there are any connections on this node, and delete
	// them before this object is gone.

	while (connectTO.size())
	{
		this->disconnect(connectTO[0]->sink->id->s_name);	
	}
	
	while (connectFROM.size())
    {
        if (connectFROM[0]->source)
            connectFROM[0]->source->disconnect(this->id->s_name);
	}
	
}

// *****************************************************************************
/*
void DSPNode::callbackUpdate()
{

	osg::Matrix myMatrix = osg::computeLocalToWorld(this->currentNodePath);
	
	if (this->_globalMatrix != myMatrix)
	{
		this->_globalMatrix = myMatrix;
		osg::Vec3 myPos = myMatrix.getTrans();
		osg::Vec3 myRot = Vec3inDegrees(QuatToEuler(myMatrix.getRotate()));
		
		BROADCAST(this, "sffffff", "global6DOF", myPos.x(), myPos.y(), myPos.z(), myRot.x(), myRot.y(), myRot.z());
	}
}
*/


// *****************************************************************************

SoundConnection *DSPNode::getConnection(DSPNode *snk)
{
	if (snk)
	{	
        std::vector<SoundConnection*>::iterator iter;
		for (iter = this->connectTO.begin(); iter != this->connectTO.end(); iter++)
		{
			if ((*iter)->sink == snk) return (*iter);
		}
	}
	
	return NULL;
}

SoundConnection *DSPNode::getConnection(const char *snk)
{
	return getConnection( dynamic_cast<DSPNode*>( sceneManager->getNode(snk) ) );
}


// *****************************************************************************
void DSPNode::connect(DSPNode *snk)
{
	// check if this connection already exists:
	if (!this->getConnection(snk))
	{
		SoundConnection *conn = new SoundConnection(this->sceneManager, this, snk);

		// add to the connection lists for each node:
		this->connectTO.push_back(conn);
		conn->sink->connectFROM.push_back(conn);
	}
	
	BROADCAST(this, "ss", "connect", snk->id->s_name);
}

// *****************************************************************************
void DSPNode::connect(const char *snk)
{
	osg::ref_ptr<DSPNode> sinkNode = dynamic_cast<DSPNode*>( sceneManager->getNode(snk) );
	if (sinkNode.valid()) this->connect(sinkNode.get());
}

void DSPNode::connectSource(const char *src)
{
	osg::ref_ptr<DSPNode> srcNode = dynamic_cast<DSPNode*>( sceneManager->getNode(src) );
	if (srcNode.valid()) srcNode->connect(this);	
}


void DSPNode::disconnect(const char *snk)
{
	// check if this connection already exists:
	SoundConnection *conn = this->getConnection(snk);

	if (conn)
	{
		/*
		// if this is the last connection for this node, then disactivate it:
		if (this->connectTO.empty() && this->connectFROM.empty())
			this->setActive(0);
		
		// also check the sink:
		if (conn->sink->connectTO.empty() && conn->sink->connectFROM.empty())
			conn->sink->setActive(0);
		 */
		
		// remove it from the connectTO list:		
        std::vector<SoundConnection*>::iterator iter;
		for (iter = connectTO.begin(); iter != connectTO.end(); iter++)
		{
			if ((*iter) == conn)
			{
				connectTO.erase(iter);
				break;
			}
		}
		
		// and remove it from the sink's connectFROM list:
		for (iter = conn->sink->connectFROM.begin(); iter != conn->sink->connectFROM.end(); iter++)
		{
			if ((*iter) == conn)
			{
				conn->sink->connectFROM.erase(iter);
				break;
			}
		}
		
		
		// now delete the actual object:
		delete conn;

		BROADCAST(this, "ss", "disconnect", snk);
	} else { 
        std::cout << "oops. couldn't find connection: " << this->id->s_name << " -> " << snk << std::endl;
    }
}


// *****************************************************************************
// *****************************************************************************



void DSPNode::setActive (int i)
{
	active = (bool)i;
	BROADCAST(this, "si", "setActive", (int)active);
}

void DSPNode::setPlugin (const char *newPlugin)
{	
	plugin = std::string(newPlugin);
	BROADCAST(this, "ss", "setPlugin", plugin.c_str());
}

/*
void DSPNode::connectionMsg (char *snkName, char *method, float value)
{
	SoundConnection *conn = sceneManager->getConnection(this->id->s_name, snkName);
	std::string theMethod = string(method);
	
	if (conn)
	{
		bool success = true;
		
		if (theMethod=="setThru")
			conn->setThru((bool) value);
		else if (theMethod=="setDistanceEffect")
			conn->setDistanceEffect(value);
		else if (theMethod=="setRolloffEffect")
			conn->setRolloffEffect(value);
		else if (theMethod=="setDopplerEffect")
			conn->setDopplerEffect(value);
		else if (theMethod=="setDiffractionEffect")
			conn->setDiffractionEffect(value);
		else
		{
			success = false;
			std::cout << "ERROR: connectionMsg (" << snkName << " " << method << " " << value << ") has unknown method." << std::endl;
		}
		
		if (success)
		{
			BROADCAST(this, "sssf", "connectionMsg", snkName, method, value);
		}
	}
	
}
*/


// *****************************************************************************

std::vector<lo_message> DSPNode::getState() const
{
	// inherit state from base class
	std::vector<lo_message> ret = GroupNode::getState();
	
	lo_message msg;
	
	msg = lo_message_new();
	lo_message_add(msg, "si", "setActive",(int) this->active);
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "ss", "setPlugin", plugin.c_str());
	ret.push_back(msg);
	
	/*
	if (connectTO.size())
	{
		msg = lo_message_new();
		lo_message_add_string(msg, "connectedTo");
		for (int i=0; i<connectTO.size(); i++)
			lo_message_add_string(msg, (char*)connectTO[i]->sink->id->s_name);
		ret.push_back(msg);
	}
	*/
	
	for (int i=0; i<connectTO.size(); i++)	
	{
		msg = lo_message_new();
		lo_message_add(msg, "ss", "connect", (char*)connectTO[i]->sink->id->s_name);
		ret.push_back(msg);
	}
	
	
	
	return ret;
}
