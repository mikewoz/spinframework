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

#include "SoundConnection.h"
#include "DSPNode.h"
#include "SceneManager.h"
#include "spinApp.h"
#include "spinBaseContext.h"
#include <iostream>

// *****************************************************************************
// constructors:

SoundConnection::SoundConnection (SceneManager *s, osg::ref_ptr<DSPNode> src, osg::ref_ptr<DSPNode> snk)
{
	
	sceneManager = s;
	
	id = gensym( (std::string(src->id->s_name) + "-" + std::string(snk->id->s_name) + ".conn" ).c_str() );
	//id->s_thing = this; // can't do this because SoundConnection doesn't extend osg::Object

	// set pointers:
	source = src.get();
	sink = snk.get();

	// set default parameter values:
	thru = false;
	distanceEffect = 100.0;
	rolloffEffect = 100.0;
	dopplerEffect = 100.0;
	diffractionEffect = 100.0;
	//proximityEffect = 0.0;
	//headEffect = 0.0;

	
	// register with OSC parser:
    std::string oscPattern = "/SPIN/" + sceneManager->sceneID + "/" + std::string(id->s_name);
    std::vector<lo_server>::iterator it;
    for (it = spinApp::Instance().getContext()->lo_rxServs_.begin(); it != spinApp::Instance().getContext()->lo_rxServs_.end(); ++it)
    {
    	lo_server_add_method((*it), oscPattern.c_str(),
            NULL, spinBaseContext::connectionCallback, (void*)this);
    }

	lo_server_add_method(spinApp::Instance().getContext()->lo_tcpRxServer_,
	                     oscPattern.c_str(),
	                     NULL,
	                     spinBaseContext::connectionCallback,
	                     (void*)this);
	
	// store pointer to sceneManager
	sceneManager = s;
	
	// broadcast the creation of this connection
	SCENE_MSG("sss", "createNode", id->s_name, "SoundConnection");
}





// *****************************************************************************
// destructor

SoundConnection::~SoundConnection()
{	
	//std::cout << "In SoundConnection destructor... id: " << this->id->s_name << std::endl;
	
	
	// broadcast the delete message of this connection
	SCENE_MSG("ss", "deleteNode", id->s_name);

    std::string oscPattern = "/SPIN/" + sceneManager->sceneID + "/" + std::string(id->s_name);
    std::vector<lo_server>::iterator it;
    for (it = spinApp::Instance().getContext()->lo_rxServs_.begin(); it != spinApp::Instance().getContext()->lo_rxServs_.end(); ++it)
    {
    	lo_server_del_method((*it), oscPattern.c_str(), NULL);
    }

	lo_server_del_method(spinApp::Instance().getContext()->lo_tcpRxServer_, oscPattern.c_str(), NULL);
	
	//id->s_thing = 0;

}


/*
// *****************************************************************************
// Computation of physical modelling:

void SoundConnection::updateSharedSpaces()
{
	vector<SoundConnection*>::iterator iter1, iter2;
	SoundConnection *conn;

	// build list of sharedSpaces:
	
	if (connectionType==NODE_TO_SPACE)
	{
		// If this is a NODE_TO_SPACE, then we need to check all the sinks that
		// the space is connected to, in order to see if a connection exist from
		// the node. If a connection exists, then we add the space to the list
		// of sharedSpaces 
		for (iter1 = sink->connectTO.begin(); iter1 != sink->connectTO.end(); iter1++) // for each node that the space is connected to
		{
			if ((*iter1)->connectionType == SPACE_TO_NODE) // if the space is connected to a sink soundNode
			{
				for (iter2 = (*iter1)->sink->connectFROM.begin(); iter2 != (*iter1)->sink->connectFROM.end(); iter2++) // look at all the source nodes for that soundNode
				{
					if ((*iter2)->source == this->source) // if the source node is the same as the node in this connection
					{
						conn = nodeManager->getConnection(source->id, (*iter1)->sink->id);
						conn->sharedSpaces.push_back( dynamic_cast<SoundSpace*>(sink.get()) ); // add the shared space
						break;
					}
				}
			}
		}
		
	} else if (connectionType==SPACE_TO_NODE)
	{
		for (iter1 = source->connectFROM.begin(); iter1 != source->connectFROM.end(); iter1++) // for each node that the space is connected from
		{
			if ((*iter1)->connectionType == NODE_TO_SPACE) // if the connection comes from a source soundNode
			{
				for (iter2 = (*iter1)->source->connectTO.begin(); iter2 != (*iter1)->source->connectTO.end(); iter2++) // look at all the sink nodes for that source
				{
					if ((*iter2)->sink == this->sink) // if any of those sink nodes is the same as this connection's sink
					{
						conn = nodeManager->getConnection( (*iter1)->source->id, sink->id );
						conn->sharedSpaces.push_back( dynamic_cast<SoundSpace*>(source.get()) ); // add the shared space
						break;
					}
				}
			}
		}
		
	} else {
		
		// For a NORMAL connection, we'll just look at all the nodes that the source
		// is connected to. If it's a soundspace, then we'll look at all the nodes
		// that the space is connected to
		
		for (iter1 = source->connectTO.begin(); iter1 != source->connectTO.end(); iter1++) // look at all the nodes that the source is connected to
		{
			if ((*iter1)->connectionType == NODE_TO_SPACE) // if it's connected to a soundspace
			{
				for (iter2 = (*iter1)->sink->connectTO.begin(); iter2 != (*iter1)->sink->connectTO.end(); iter2++) // look at all the nodes that the soundspace is connected to
				{
					if ((*iter2)->sink == this->sink) // if a soundspace is connected to a sink that is the same as this connection's sink
					{
						sharedSpaces.push_back( dynamic_cast<SoundSpace*>( (*iter1)->sink.get() ) ); // add it
					}
				}
			}
		}

		
	}
}
	


void SoundConnection::computeWorldMatrices()
{
	srcMatrix = osg::computeLocalToWorld(source->currentNodePath);
	snkMatrix = osg::computeLocalToWorld(sink->currentNodePath);
}

	
void SoundConnection::computeDistance()
{	
	
	// translation components from matrices:
	osg::Vec3 srcPos = srcMatrix.getTrans();
	osg::Vec3 snkPos = snkMatrix.getTrans();
	
	// get a pointer to the space (if there is one):
	osg::ref_ptr<SoundSpace> space;
	if (connectionType==NODE_TO_SPACE) space = dynamic_cast<SoundSpace*>( sink.get() );
	else if (connectionType==SPACE_TO_NODE) space = dynamic_cast<SoundSpace*>( source.get() );
	
	if (space.valid())
	{

#ifdef AS_GRAPHICAL
		// create a line segment between the source and sink:
		osg::LineSegment* connectionLineSegment = new osg::LineSegment();
		connectionLineSegment->set( srcPos, snkPos );

		// Reset the IntersectVisitor, add line segment to test for intersections, and
		// then start traversal by calling the accept() method:
		space->soundSpaceIntersectVisitor.reset();
		space->soundSpaceIntersectVisitor.addLineSegment(connectionLineSegment);
		//space->soundSpaceIntersectVisitor.setTraversalMask(2);
		space->mainTransform->accept(space->soundSpaceIntersectVisitor);
		osgUtil::IntersectVisitor::HitList hits = space->soundSpaceIntersectVisitor.getHitList(connectionLineSegment);

		// find distance by examining intersections:

		//std::cout << "intersect test: " << source->id->s_name << "=(" << srcPos.x() << "," << srcPos.y() << "," << srcPos.z() << "), " << sink->id->s_name << "=(" << snkPos.x() << "," << snkPos.y() << "," << snkPos.z() << ")" << std::endl;
		
		if (hits.empty()) // this means that the node is inside the space
		{
			isInside = 1.0;
			connection_vector = osg::Vec3(0.0, 0.0, 0.0);
			//std::cout << "  no intersection; node is inside space" << std::endl;
			
		} else { // this means we are outside the space
			
			isInside = 0.0;
			osgUtil::Hit hit = hits.front();
			osg::Vec3d hitPoint = hit.getWorldIntersectPoint();
			//std::cout << "  hitPoint=(" << hitPoint.x() << "," << hitPoint.y() << "," << hitPoint.z() << ")" << std::endl;
			//osg::Vec3d hitNormal = hit.getWorldIntersectNormal();
			if (connectionType==NODE_TO_SPACE) connection_vector = hitPoint - srcPos;
			else connection_vector = snkPos - hitPoint;
		}
#else
		// TODO: perform distance computation for simple shapes (not requiring IntersectVisitor)
		isInside = 0.0;
		connection_vector = snkPos - srcPos;
#endif

	} else { // this is just a regular connection between two spaces
		connection_vector = snkPos - srcPos;
	}
	
	// final computation:
	//distance = connection_vector.length();
	distanceScaler = 1 / (1.0 + pow((double)connection_vector.length(),(double)distanceEffect*.01));
	
}

void SoundConnection::computeRolloff()
{
	
	//std::cout << " SoundConnection::computeRolloff (" << source->id->s_name << " - " << sink->id->s_name << ")" << std::endl;
	// A rolloff table contains gain values for different angles of incidence.
	//
	// Incidence, for that matter, is the angular between an object's direction
	// vector, and the vector connecting the two nodes.
	//
	// Incidence of 0 means that the connection_vector is perfectly aligned with
	// the direction vector (ie, source is pointing directly at the sink, or vice versa).

	// get matrices:
	//osg::Matrix srcMatrix = source->getWorldCoords();
	//osg::Matrix snkMatrix = sink->getWorldCoords();
	
	// Let's start by getting quaternion representations for the source and sink nodes:
	osg::Quat srcQuat, snkQuat;
	srcMatrix.get(srcQuat); // should be srcQuat=srcMatrix.getRotate(), shouldn't it? nope. don't think so.
	snkMatrix.get(snkQuat);
	
	// let's also compute the orientations projected on the (local) horizontal and vertical plane:
	// (ie, azimuth and elevation respectively)
	osg::Vec3 src_dir   = srcQuat * osg::Vec3(0,1,0);
	osg::Vec3 src_right = srcQuat * osg::Vec3(1,0,0);
	osg::Vec3 src_up    = srcQuat * osg::Vec3(0,0,1);
	osg::Vec3 snk_dir   = snkQuat * osg::Vec3(0,1,0);
	osg::Vec3 snk_right = snkQuat * osg::Vec3(1,0,0);
	osg::Vec3 snk_up    = snkQuat * osg::Vec3(0,0,1);
	
	// THOUGHT: the incidence with spaces is NOT CORRECT... we need to consider the bound
	
	// incidence (radians) between source and the connection_vector:
	srcIncidence = (t_float) AngleBetweenVectors(src_dir, connection_vector);
	srcIncidenceAzim = (t_float) (osg::PI/2) - AngleBetweenVectors(src_right, connection_vector);
	srcIncidenceElev = (t_float) (osg::PI/2) - AngleBetweenVectors(src_up, connection_vector);

	// incidence (radians) between sink and the connection_vector:
	snkIncidence = (t_float) AngleBetweenVectors(osg::Vec3(0,0,0)-snk_dir, connection_vector);		
	snkIncidenceAzim = (t_float) (osg::PI/2) - AngleBetweenVectors(snk_right, -connection_vector);
	snkIncidenceElev = (t_float) (osg::PI/2) - AngleBetweenVectors(snk_up, -connection_vector);
	
	// Now, note that rolloff & incidence only applies to soundNodes (not soundSpaces),
	// so we check if this connection involves any soundSpaces, and if so, we will set
	// their rolloffScaler to 1.0 (unity). Whereas the soundNodes will have a
	// rolloffScaler based on their incidence and rolloff table.

	// Note: the spread parameter is used to distort the sampling of the rolloff
	// table. A spread of 1.0 means we sample it precisely, while a spread
	// of 2.0 means we sample twice as fast (see ss_rolloffTable for more info).
	
	t_float srcGain, snkGain;
	osg::ref_ptr<SoundNode> n;
	
	if (connectionType==NODE_TO_SPACE) {
		n = dynamic_cast<SoundNode*>( source.get() );
		srcGain = n->rolloff->getValue( (srcIncidence * n->_spread) / osg::PI );
		snkGain = 1.0;
	} else if (connectionType==SPACE_TO_NODE) {
		n = dynamic_cast<SoundNode*>( sink.get() );
		snkGain = n->rolloff->getValue( (snkIncidence * n->_spread) / osg::PI );
		srcGain = 1.0;
	} else {
		n = dynamic_cast<SoundNode*>( source.get() );
		srcGain = n->rolloff->getValue( (srcIncidence * n->_spread) / osg::PI );
		n = dynamic_cast<SoundNode*>( sink.get() );
		snkGain = n->rolloff->getValue( (snkIncidence * n->_spread) / osg::PI );
	}
	
	rolloffScaler = (double) (1.0 - (.01*rolloffEffect  * (1.0 - srcGain*snkGain)));
	
}

void SoundConnection::computeSeparation()
{
	//std::cout << " SoundConnection::computeSeparation (" << source->id->s_name << " - " << sink->id->s_name << ")" << std::endl;
	// Here we go through the list of sharedSpaces, and see if the source and sink
	// are on opposite sides of any space (ie, isInside flag is opposite). Hence,
	// this should ideally be called AFTER the computeDistance() function.
	isSeparated = 0.0;
	occludingSpace = gensym("NULL");
	
	vector< osg::ref_ptr<SoundSpace> >::iterator iter;
	SoundConnection *space2node, *node2space;
	
	//std::cout << "testing " << sharedSpaces.size() << " spaces to see if source (" << source->id->s_name << ") is separated from sink (" << sink->id->s_name << ")" << std::endl;
	for (iter = sharedSpaces.begin(); iter != sharedSpaces.end(); iter++)
	{
		//std::cout << "testing space [" << (*iter)->id->s_name << "]:" << std::endl;
		node2space = nodeManager->getConnection(source->id, (*iter)->id);
		space2node = nodeManager->getConnection((*iter)->id, sink->id);
		if ( (node2space!=NULL) && (space2node!=NULL) )
		{
			if (space2node->isInside != node2space->isInside)
			{
				isSeparated = 1.0;
				occludingSpace = (*iter)->id;
				break;
			}
			//std::cout << "  sourceInside=" << node2space->isInside << ", sinkInside=" << space2node->isInside << ", thus separated=" << isSeparated << std::endl;
		}
	}
	
}
*/

// *****************************************************************************


void SoundConnection::setThru (int newvalue)
{
	if (newvalue > 0) this->thru = true;
	else this->thru = false;
	
	BROADCAST(this, "si", "setThru", this->thru);
}

void SoundConnection::setDistanceEffect (float newvalue)
{
	this->distanceEffect = newvalue;
	BROADCAST(this, "sf", "setDistanceEffect", this->distanceEffect);
}

void SoundConnection::setRolloffEffect (float newvalue)
{
	this->rolloffEffect = newvalue;
	BROADCAST(this, "sf", "setRolloffEffect", this->rolloffEffect);
}

void SoundConnection::setDopplerEffect (float newvalue)
{
	this->dopplerEffect = newvalue;
	BROADCAST(this, "sf", "setDopplerEffect", this->dopplerEffect);
}

void SoundConnection::setDiffractionEffect (float newvalue)
{
	this->diffractionEffect = newvalue;
	BROADCAST(this, "sf", "setDiffractionEffect", this->diffractionEffect);
}


// *****************************************************************************

void SoundConnection::debug()
{
	lo_arg **args;
	int i, argc;
	char *argTypes;
	
	std::cout << "****************************************" << std::endl;
	std::cout << "********** CONNECTION  DEBUG: **********" << std::endl;
	
	std::cout << "\nSoundConnection [" << this->id << "]:" << std::endl;
	
    std::vector<lo_message> nodeState = this->getState();
    std::vector<lo_message>::iterator nodeStateIterator;
	for (nodeStateIterator = nodeState.begin(); nodeStateIterator != nodeState.end() ; nodeStateIterator++)
	{ 
	    argTypes = lo_message_get_types(*nodeStateIterator);
	    argc = lo_message_get_argc(*nodeStateIterator);
	    args = lo_message_get_argv(*nodeStateIterator);

	    std::cout << "  ";
	    for (i = 0; i<argc; i++) {
		    std::cout << " ";
	    	if (lo_is_numerical_type((lo_type)argTypes[i]))
	    	{
	    		std::cout << (float) lo_hires_val( (lo_type)argTypes[i], args[i] );
	    	} else if (strlen((char*) args[i])) {
	    		std::cout << (char*) args[i];
	    	} else {
	    		std::cout << "NULL";
	    	}
	    }
	    std::cout << std::endl;
	}
	
}

// *****************************************************************************

/* Note that getState() and stateDump() work differently for connections. They
 * are always send with respect to the source node of the connection.
 * 
 */
std::vector<lo_message> SoundConnection::getState () const
{
	std::vector<lo_message> ret;
	lo_message msg;
	
	msg = lo_message_new();
	lo_message_add(msg, "ss", "src", source->id->s_name);
	ret.push_back(msg);
	
	msg = lo_message_new();
	lo_message_add(msg, "ss", "snk", sink->id->s_name);
	ret.push_back(msg);
	
	msg = lo_message_new();
	lo_message_add(msg, "sf", "setThru", (float) this->getThru());
	ret.push_back(msg);
	
	msg = lo_message_new();
	lo_message_add(msg, "sf", "setDistanceEffect", this->getDistanceEffect());
	ret.push_back(msg);
	
	msg = lo_message_new();
	lo_message_add(msg, "sf", "setRolloffEffect", this->getRolloffEffect());
	ret.push_back(msg);
	
	msg = lo_message_new();
	lo_message_add(msg, "sf", "setDopplerEffect", this->getDopplerEffect());
	ret.push_back(msg);
	
	msg = lo_message_new();
	lo_message_add(msg, "sf", "setDiffractionEffect", this->getDiffractionEffect());
	ret.push_back(msg);
	
	return ret;
}
	
// *****************************************************************************
void SoundConnection::stateDump ()
{

    spinApp::Instance().NodeBundle(this->id, this->getState());
	
	/*
	vector<lo_message> nodeState = this->getState();

	vector<lo_message>::iterator iter = nodeState.begin();
	while (iter != nodeState.end())
	{
		//sceneManager->sendNodeMessage(source->id, (*iter));
		sceneManager->sendNodeMessage(this->id, (*iter));
		nodeState.erase(iter); //note: iterator automatically advances after erase()
	}
	*/
}


void SoundConnection::stateDump (lo_address addr)
{
    spinApp::Instance().NodeBundle(this->id, this->getState(), addr);
}
