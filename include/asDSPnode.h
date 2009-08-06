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
		void connect(char *snk);
		void disconnect(char *snk);
		void setActive (int i);
		void setDSP (const char *filename);
		
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
		//virtual void stateDump() { asReferenced::stateDump(); };
		
		
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
