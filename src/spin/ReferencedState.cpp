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

#include <osg/StateSet>
#include <osg/StateAttribute>

#include <iostream>

#include "SceneManager.h"
#include "ReferencedState.h"



using namespace std;





// *****************************************************************************
// constructor:
ReferencedState::ReferencedState(SceneManager *s, const char *initID)
{
	// concatenate
	//string thisID = string(n->id->s_name)+"/"+initID;

	id = gensym(initID);
	id->s_thing = this;
	id->s_type = REFERENCED_STATE;
	
	sceneManager = s;
	
	classType = "ReferencedState";
	
	this->setName(string(id->s_name) + ".ReferencedState");
	
	// We need to set up a callback. This should be on the topmost node, so that during node
	// traversal, we update our parameters before anything is drawn.
	this->setUserData( dynamic_cast<osg::Referenced*>(this) );
	this->setUpdateCallback(new ReferencedState_callback);
	
}

// destructor
ReferencedState::~ReferencedState()
{
	// This will be called when the parent node is killed
	
	// clear the stateset
	this->clear();
	
	// unregister from sceneManager:
	sceneManager->unregisterState(this);

	/*
	// set the UserData to NULL (removing the ref_ptr):
	setUserData( NULL );
	*/
	
	// finally, by nulling the ref_ptr in s_thing, we should have removed all
	// references to this object, so OSG can clean up
	id->s_thing = 0;

}

// *****************************************************************************
void ReferencedState::updateCallback()
{
    // derived classes can do updates here   
}


void ReferencedState::replace(osg::StateSet *ss)
{
	//vid->addParent((*itr)->getParent(0));
	//(*itr)->removeParent(*itr)->getParent(0));

	osg::StateSet::ParentList::iterator itr;
	osg::StateSet::ParentList parents = ss->getParents();	
	
	for (itr=parents.begin(); itr!=parents.end(); ++itr)
	{
		osg::Node *node = dynamic_cast<osg::Node*>(*itr);
		if (node)
		{
			node->setStateSet(this);
		}
		else {
			osg::Drawable *drawable = dynamic_cast<osg::Drawable*>(*itr);
			if (drawable) drawable->setStateSet(this);
		}
	}

}


// *****************************************************************************
void ReferencedState::debug()
{
	lo_arg **args;
	int i, argc;
	char *argTypes;

	std::cout << "****************************************" << std::endl;
	std::cout << "************* STATE DEBUG: *************" << std::endl;

	std::cout << "\nReferencedState: " << id->s_name << ", type: " << classType << std::endl;

	
	std::cout << "   Shared by:";
	for (i=0; i<getNumParents(); i++)
	{
		std::cout << " " << getParent(i)->getName();
	}
	std::cout << std::endl;
	
	
	vector<lo_message> nodeState = this->getState();
	vector<lo_message>::iterator nodeStateIterator;
	for (nodeStateIterator = nodeState.begin(); nodeStateIterator != nodeState.end(); ++nodeStateIterator)
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
	
	BROADCAST(this, "s", "debug");
}

// *****************************************************************************
std::vector<lo_message> ReferencedState::getState ()
{
	std::vector<lo_message> ret;

	return ret;
}


void ReferencedState::stateDump()
{
	sceneManager->sendNodeBundle(this->id, this->getState());
}