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


#ifndef __ASSOUNDCONNECTION_H
#define __ASSOUNDCONNECTION_H

#include <osg/Node>
#include "asGlobals.h"
#include "asDSPnode.h"
#include "asSceneManager.h"

enum connectionType { NORMAL, NODE_TO_SPACE, SPACE_TO_NODE };

class asDSPnode;

/**
 * \brief An explicit connection between two sound elements (ie, subclassed from
 *        asDSPnode)
 * 
 * By default, sounds are not "connected" to each other. To specify that sound
 * should travel from one node to another, an asSoundConnection instance must be
 * made. The correct way to do this is to call the connect() method for the
 * asDSPnode that represents the source of the connection.
 */
class asSoundConnection : virtual public osg::Node
{
 
	public:
		asSoundConnection(asSceneManager *s, osg::ref_ptr<asDSPnode> src, osg::ref_ptr<asDSPnode> snk);
		~asSoundConnection();

		t_symbol *id;
		
		
		void setThru (int newvalue);
		void setDistanceEffect (float newvalue);
		void setRolloffEffect (float newvalue);
		void setDopplerEffect (float newvalue);
		void setDiffractionEffect (float newvalue);
		
		int   getThru() { return (int) thru; };
		float getDistanceEffect() { return distanceEffect; };
		float getRolloffEffect() { return rolloffEffect; };
		float getDopplerEffect() { return dopplerEffect; };
		float getDiffractionEffect() { return diffractionEffect; };

		
		void debug();
		virtual std::vector<lo_message> getState();
		virtual void stateDump();
		
		// ************
	
		
		// These are pointers to the nodes for this connection:
		//osg::ref_ptr<asDSPnode> source, sink;
		asDSPnode *source, *sink;
	
		
	private:
		
		// The modifiable parameters of the connection (allows for bending the rules
		// of physical modelling):
		bool thru;
		float distanceEffect;
		float rolloffEffect;
		float dopplerEffect;
		float diffractionEffect;
		
		
		asSceneManager *sceneManager;

};

#endif
