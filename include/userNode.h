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

#ifndef __USERNODE_H
#define __USERNODE_H

#include "asGlobals.h"
#include "asBasicNode.h"



/**
 * \brief Represents a user in the scene.
 *
 * This class is used to differentiate users from scene content. The subgraph
 * may contain:
 * - cameras that render according the user's perspective
 * - a graphical avatar to provide an objective representation of the user
 * - various soundNodes that correspond the ther users loudspeaker setup
 * - etc.
 *
 * It is important to note that anything attached to a userNode's subgraph will
 * not be saved with the scene.
 */
class userNode : public asBasicNode
{

	public:

		userNode(asSceneManager *sceneManager, char *initID);
		virtual ~userNode();

		// SET methods:
		void setDescription (const char *s);


		// GET methods:
		const char* getDescription() { return _description.c_str(); }


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


		// We must redefine any methods from out base class (asBasicNode) so
		// that osg::Introspection will see them. This is really only necessary
		// for methods for which we want handlers (OSC/WX/etc).




	private:

		std::string _description;

};


#endif
