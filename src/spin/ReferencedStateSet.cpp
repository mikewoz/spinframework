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

#include "spinApp.h"
#include "spinBaseContext.h"
#include "SceneManager.h"
#include "ReferencedStateSet.h"



using namespace std;


extern pthread_mutex_t sceneMutex;

// *****************************************************************************
// constructor:
ReferencedStateSet::ReferencedStateSet(SceneManager *s, const char *initID)
{
	// concatenate
	//string thisID = string(n->id->s_name)+"/"+initID;

	
	id = gensym(initID);
	id->s_thing = this;
	id->s_type = REFERENCED_STATESET;
	
	sceneManager = s;
	
	classType = "ReferencedStateSet";
	
	this->setName(string(id->s_name) + ".ReferencedStateSet");
	
	// We need to set up a callback. This should be on the topmost node, so that during node
	// traversal, we update our parameters before anything is drawn.
	this->setUserData( dynamic_cast<osg::Referenced*>(this) );
	this->setUpdateCallback(new ReferencedStateSet_callback);
	
}

// destructor
ReferencedStateSet::~ReferencedStateSet()
{

}

// *****************************************************************************
void ReferencedStateSet::updateCallback()
{
    // derived classes can do updates here   
}


void ReferencedStateSet::removeFromScene()
{
	//pthread_mutex_lock(&sceneMutex);
	
	osg::StateSet::ParentList::iterator itr;
	osg::StateSet::ParentList parents = this->getParents();	
	
	for (itr=parents.begin(); itr!=parents.end(); ++itr)
	{
		osg::Node *node = dynamic_cast<osg::Node*>(*itr);
		if (node)
		{
			node->setStateSet(0);
		}
		else {
			osg::Drawable *drawable = dynamic_cast<osg::Drawable*>(*itr);
			if (drawable) drawable->setStateSet(0);
		}
	}
	
	this->setUserData( NULL );
	
	//pthread_mutex_unlock(&sceneMutex);
}



void ReferencedStateSet::replace(osg::StateSet *ss)
{
	// first, try to inherit as much of the StateSet's attributes as possible:
	
	// but make sure to remove texture attribute 0 first:
	//osg::StateAttribute* attr = stateset->getTextureAttribute(0,osg::StateAttribute::TEXTURE);
	
	//ss->removeTextureAttribute(0, osg::StateAttribute::TEXTURE);
	
	this->merge(*ss); // oops. this will replace our textures (unless the OVERRIDE flag is set!)


	//this->setModeList(ss->getModeList());

	/*
	osg::StateSet::AttributeList attribs = ss->getAttributeList();
	osg::StateSet::AttributeList::iterator attr;
	for (attr=attribs.begin(); attr!=attribs.end(); ++attr)
	{
		osg::StateAttribute *a = (*attr).second.first;
		std::cout << "attrib: " << a->className() <<  std::endl;
		//std::cout << "attrib: " << (int)(*attr).first.first << " " << (*attr).second.first->getName <<  std::endl;
	}
	*/
	
	
	// now replace ss in each of his parents with our stateset:
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
void ReferencedStateSet::debug()
{
	lo_arg **args;
	int argc;
	char *argTypes;

	std::cout << "****************************************" << std::endl;
	std::cout << "************* STATE DEBUG: *************" << std::endl;

	std::cout << "\nReferencedStateSet: " << id->s_name << ", type: " << classType << std::endl;

	
	std::cout << "   Shared by:";
	for (unsigned i = 0; i < getNumParents(); i++)
		std::cout << " " << getParent(i)->getName();
	std::cout << std::endl;

	//osg::ref_ptr<ReferencedStateSet> test = this;
	//std::cout << "ref_count=" << test->getReferenceCount() << std::endl;
	
	
	vector<lo_message> nodeState = this->getState();
	vector<lo_message>::iterator nodeStateIterator;
	for (nodeStateIterator = nodeState.begin(); nodeStateIterator != nodeState.end(); ++nodeStateIterator)
	{
	    argTypes = lo_message_get_types(*nodeStateIterator);
	    argc = lo_message_get_argc(*nodeStateIterator);
	    args = lo_message_get_argv(*nodeStateIterator);

	    std::cout << "  ";
	    for (int i = 0; i < argc; i++) {
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
std::vector<lo_message> ReferencedStateSet::getState ()
{
	std::vector<lo_message> ret;

	return ret;
}


void ReferencedStateSet::stateDump()
{
    spinApp::Instance().NodeBundle(this->id, this->getState());
}


void ReferencedStateSet::stateDump(lo_address addr)
{
    spinApp::Instance().NodeBundle(this->id, this->getState(), addr);
}
