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

#ifndef __ReferencedStateSet_H
#define __ReferencedStateSet_H

#include "spinUtil.h"
#include "libloUtil.h"

#include <osg/StateSet>

// forward declaration of SceneManager
class SceneManager;


/**
 * \brief ReferencedStateSet is the base class for StateSets attached to nodes
 */

class ReferencedStateSet : virtual public osg::StateSet
{

public:

	ReferencedStateSet(SceneManager *sceneManager, const char *initID);
	~ReferencedStateSet();

	virtual void updateCallback();

	/**
	 * Abstract method getPath needs to be implemented
	 */
	virtual const char *getPath() = 0;

	/**
	* Remove this stateset from all parents... essentially destroying the state,
	* since no reference to it will exist anymore, and OSG will kill it.
	*/
	void removeFromScene();

	/**
	 * Replaces a StateSet in the scene graph with this one. ie, goes through
	 * all parents of the provided stateset and replaces the object's state with
	 * this.
	 */ 
	void replace(osg::StateSet *ss);
	
	virtual void debug();
	
	/**
	 * Just like a ReferencedNode, each subclass of ReferencedStateSet must
	 * override the getState() method to pass it's current state.
	 */
	virtual std::vector<lo_message> getState();

	/**
	 * StateDump() is a request to broadcast the node state via SceneManager.
	 */
	virtual void stateDump();
	virtual void stateDump(lo_address txAddr);
	
	
	SceneManager *sceneManager;
	t_symbol *id;
	std::string classType;

private:
	


};

typedef std::vector< osg::ref_ptr<ReferencedStateSet> > stateListType;

class ReferencedStateSet_callback : public osg::StateSet::StateSet::Callback
{
	public:
		
		virtual void operator()(osg::StateSet* ss, osg::NodeVisitor* /*nv*/)
		{
			osg::ref_ptr<ReferencedStateSet> thisState = dynamic_cast<ReferencedStateSet*> (ss->getUserData());

			if (thisState.valid())
			{
				thisState->updateCallback();
			}
		}
};


#endif
