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

#include "asDSPnode.h"
#include "asReferenced.h"
#include "osgUtil.h"

#include "asSceneManager.h"

//extern asSceneManager *sceneManager;

using namespace std;

// *****************************************************************************
// constructor:
asDSPnode::asDSPnode (asSceneManager *sceneManager, char *initID) : asReferenced(sceneManager, initID)
{

	nodeType = "asDSPnode";
	
	// connection stuff:
	connectTO.clear();
	connectFROM.clear();
	
	active = 1;
	
	dsp = "empty~";	
	
}

// *****************************************************************************
// destructor
asDSPnode::~asDSPnode()
{
	//std::cout << "In asDSPnode destructor... node: " << this->id->s_name << std::endl;
	
	
	// we should check if there are any connections on this node, and delete
	// them before this object is gone.

	while (connectTO.size())
	{
		this->disconnect(connectTO[0]->sink->id->s_name);	
	}
	
	while (connectFROM.size())
	{
		connectFROM[0]->source->disconnect(this->id->s_name);
	}
	
}

// *****************************************************************************
void asDSPnode::callbackUpdate()
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

// *****************************************************************************
asSoundConnection *asDSPnode::getConnection(char *snk)
{
	vector<asSoundConnection*>::iterator iter;
	for (iter = this->connectTO.begin(); iter != this->connectTO.end(); iter++)
	{
		if ((*iter)->sink->id == gensym(snk)) return (*iter);
	}
	
	return NULL;
}


// *****************************************************************************
void asDSPnode::connect(char *snk)
{
	// check if this connection already exists:
	if (this->getConnection(snk)) return;
	
	osg::ref_ptr<asDSPnode> sinkNode = dynamic_cast<asDSPnode*>( sceneManager->getNode(snk) );
	if (sinkNode.valid())
	{

		asSoundConnection *conn = new asSoundConnection(this->sceneManager, this, sinkNode);
		
		// add to the connection lists for each node:
		this->connectTO.push_back(conn);
		conn->sink->connectFROM.push_back(conn);
		
		BROADCAST(this, "ss", "connect", snk);
		
	}
}

void asDSPnode::disconnect(char *snk)
{
	
	// check if this connection already exists:
	asSoundConnection *conn = this->getConnection(snk);
	

	if (conn)
	{
		//std::cout << "Disconnecting " << conn->id->s_name << std::endl;
		
		/*
		// if this is the last connection for this node, then disactivate it:
		if (this->connectTO.empty() && this->connectFROM.empty())
			this->setActive(0);
		
		// also check the sink:
		if (conn->sink->connectTO.empty() && conn->sink->connectFROM.empty())
			conn->sink->setActive(0);
		 */
		
		// remove it from the connectTO list:		
		vector<asSoundConnection*>::iterator iter;
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
	}
}


// *****************************************************************************
// *****************************************************************************



void asDSPnode::setActive (int i)
{
	active = (bool)i;
	BROADCAST(this, "si", "setActive", (int)active);
}

void asDSPnode::setDSP (char *newDSP)
{	
	dsp = std::string(newDSP);
	BROADCAST(this, "ss", "setDSP", dsp.c_str());
}

/*
void asDSPnode::connectionMsg (char *snkName, char *method, float value)
{
	asSoundConnection *conn = sceneManager->getConnection(this->id->s_name, snkName);
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

std::vector<lo_message> asDSPnode::getState ()
{
	// inherit state from base class
	std::vector<lo_message> ret = asReferenced::getState();
	
	lo_message msg;
	
	msg = lo_message_new();
	lo_message_add(msg, "si", "setActive",(int) this->active);
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "ss", "setDSP", dsp.c_str());
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