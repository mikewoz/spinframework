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

#ifndef __ASDSPNODE_H
#define __ASDSPNODE_H

//#include <vector>
#include <osg/Referenced>

#include "asGlobals.h"
#include "asReferenced.h"
#include "asSoundConnection.h"

class asSoundConnection;

/**
 * \brief The base class for 3D audio nodes
 * 
 * This class allows for spatially localized sounds in the scene. However, this
 * should not be instantiated directly, rather, one needs to create a subclass
 * for each audio representation.
 * 
 * The main feature of this class is that it maintains the connection logic that
 * describes which node sends sound to another. The connect() and disconnect()
 * methods create the interface for managing connections for all derived classes
 */
class asDSPnode : public asReferenced
{

public:
		
		asDSPnode (asSceneManager *sceneManager, char *initID);
		virtual ~asDSPnode();
		
		virtual void callbackUpdate();
		
		
		asSoundConnection *getConnection(char *snk);

		
		// define these as virtual so subclasses can redefine them:
		virtual void connect(char *snk);
		virtual void disconnect(char *snk);
		virtual void setActive (int i);
		virtual void setDSP (char *newDSP);
		
		// for sending messages to the connections of this (source) node:
		//virtual void connectionMsg (char *snkName, char *method, float value);

		
		int getActive() { return (int)active; }
		const char* getDSP() { return dsp.c_str(); }
		
		/**
		 * We maintian 2 lists of all asSoundConnection for this node (it is
		 * redundant, but useful to have both forward and backward connection
		 * pointers).
		 */
		std::vector<asSoundConnection*> connectTO;
		/**
		 * We maintian 2 lists of all asSoundConnection for this node (it is
		 * redundant, but useful to have both forward and backward connection
		 * pointers).
		 */
		std::vector<asSoundConnection*> connectFROM;
		
		/**
		 * For each subclass of asReferenced, we override the getState() method to
		 * fill the vector with the correct set of methods for this particular node
		 */
		virtual std::vector<lo_message> getState();
		
		/**
		 * We must include a stateDump() method that simply invokes the base class
		 * method. Simple C++ inheritance is not enough, because osg::Introspection
		 * won't see it.
		 */
		virtual void stateDump() { asReferenced::stateDump(); };
		
		
private:
	
		bool active;
		
		/**
		 * dsp name (this is the name of a pd abstraction that handles the dsp):
		 */
		std::string dsp;
		
		/**
		 * This node should always broadcast global position and orientation 
		 * so that any audio spatializer software listening to messages can use
		 * the data without needing to understand and maintain a scene graph.
		 */
		osg::Matrix _globalMatrix;

};


#endif
