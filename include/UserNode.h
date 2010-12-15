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

#ifndef __UserNode_H
#define __UserNode_H


#include "GroupNode.h"
#include "ConstraintsNode.h"

#include <osg/Timer>



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
 * It is important to note that anything attached to a UserNode's subgraph will
 * not be saved with the scene.
 */
class UserNode : public ConstraintsNode
{

	public:

		UserNode(SceneManager *sceneManager, char *initID);
		virtual ~UserNode();


		/**
		 * The UserNode needs an update callback to check if ping messages are
		 * still being received. If not, the node and it's subgraph should be
		 * removed.
		 */
		virtual void callbackUpdate();

	
		/**
		 * The UserNode is used by OSG's NodeTrackerManipulator to position a
		 * camera for the user. However, NodeTrackerManipulator doesn't check if
		 * the nodepath has changed, so we override updateNodePath() and set an
		 * nodepathUpdate flag for the manipulator to see.
		 */
		virtual void updateNodePath();
		bool nodepathUpdate;
		
		
		// SET methods:
		void setDescription (const char *s);


		// GET methods:
		const char* getDescription() { return description_.c_str(); }


		/**
		* This is where you attach cameras for the user.
		*
		* The standard ViewerManipulator for SPIN derives from OSG's
		* NodeTrackerManipulator, which tracks (points to) the center of a
		* subgraph's bounding sphere. We want to keep that bound empty so that
		* we may effectively place the camera right at the UserNode's location.
		* If we attach geometry under this, the camera will point at the center
		* of that geometry instead.
		*/
		osg::Group *getCameraAttachmentNode() { return cameraAttachmentNode; }
		
		// ping message
		void ping();
		osg::Timer_t getLastPing() { return lastPing_; }

		/**
		 * For each subclass of ReferencedNode, we override the getState() method to
		 * fill the vector with the correct set of methods for this particular node
		 */
		virtual std::vector<lo_message> getState();




	private:

		bool ping_;
		osg::Timer_t lastPing_;
		std::string description_;

		osg::ref_ptr<osg::Group> cameraAttachmentNode;

};


#endif
