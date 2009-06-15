// ===================================================================
// Audioscape library for PureData
// Copyright (c) 2007
//
// Collaborators:
//    Shared Reality Lab (SRE), McGill University Centre for Intelligent Machines (CIM)
//       www.cim.mcgill.ca/sre
//    La Société des Arts Technologiques (SAT)
//       www.sat.qc.ca
//
// Project Directors:
//    Science - Jeremy R. Cooperstock (SRE/CIM)
//    Arts - Zack Settel
//
// Conception:
//    Zack Settel
//
// Development Team:
//    Mike Wozniewski (SRE/CIM): Researcher, Head Developer
//    Zack Settel: Artist, Researcher, Audio/DSP programming
//    Jean-Michel Dumas (SAT): Assistant Researcher
//    Mitchel Benovoy (SRE/CIM): Video Texture Programming
//    Stéphane Pelletier (SRE/CIM): Video Texture Programming
//    Pierre-Olivier Charlebois (SRE/CIM): Former Developer
//
// Funding by / Souventionné par:
//    Natural Sciences and Engineering Research Council of Canada (NSERC)
//    Canada Council for the Arts
//    NSERC/Canada Council for the Arts - New Media Initiative
//
// ===================================================================
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
// ===================================================================


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
