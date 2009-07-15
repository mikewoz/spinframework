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

#include <string>
#include <vector>
#include <iostream>

#include <osgText/Text>

#include "asGlobals.h"
#include "asReferenced.h"
#include "asSceneManager.h"


using namespace std;



// ***********************************************************
// constructor (one arg required: the node ID)
asReferenced::asReferenced (asSceneManager *sceneManager, char *initID)
{
	id = gensym(initID);
	id->s_thing = this;
	
	nodeType = "asReferenced";
	
	this->setName(string(id->s_name) + ".asReferenced");
	
	// set some initial symbols:
	parent = WORLD_SYMBOL;
	newParent = WORLD_SYMBOL;
	
	textFlag = false;
	
	// When children are attached to this, they get added to the attachmentNode:
	attachmentNode = this;

	// We need to set up a callback. This should be on the topmost node, so that during node
	// traversal, we update our parameters before anything is drawn.
	this->setUserData( dynamic_cast<osg::Referenced*>(this) );
	this->setUpdateCallback(new asReferenced_callback);
	
	// set initial nodepath:
	currentNodePath.clear();

	registerNode(sceneManager);

	this->setNodeMask(GEOMETRIC_NODE_MASK); // nodemask info in asGlobals.h
}

// ***********************************************************
// destructor
asReferenced::~asReferenced()
{
	//std::cout << "In asReferenced destructor... node: " << this->id->s_name << std::endl;
	
	// register with OSC parser:
	string oscPattern = "/vess/" + sceneManager->sceneID + "/" + string(id->s_name);
	lo_server_thread_del_method(sceneManager->rxServ, oscPattern.c_str(), NULL);
	
#ifdef OSCDEBUG
	std::cout << "oscParser unregistered: " << oscPattern << std::endl;
#endif
	
	id->s_thing = 0;
}


void asReferenced::registerNode(asSceneManager *s)
{
	sceneManager = s;
	mediaManager = sceneManager->mediaManager;
	
	// register with OSC parser:
	string oscPattern = "/vess/" + sceneManager->sceneID + "/" + string(id->s_name);
	lo_server_thread_add_method(sceneManager->rxServ, oscPattern.c_str(), NULL, asSceneManagerCallback_node, (void*)id);
#ifdef OSCDEBUG
	std::cout << "oscParser registered: " << oscPattern << std::endl;
#endif
}

// *****************************************************************************
void asReferenced::callbackUpdate()
{
	/*
	mail msg;
	while (netmail_get(pd_mail_id, &msg))
	{
		sceneManager->dispatcher->event(this, &msg);
	}
	*/	
}


// *****************************************************************************

void asReferenced::attach()
{
	if (this->newParent==NULL_SYMBOL) return;
	
	osg::ref_ptr<asReferenced> newParentNode = newParent->s_thing;
	
	// if the parent is invalid (which will be the case, for example, if the user
	// specified 'world' as the parent), we attach to the worldNode:
	if (!newParentNode.valid())
	{
		if (!(sceneManager->worldNode->containsNode( this ))) sceneManager->worldNode->addChild(this);
		this->newParent = WORLD_SYMBOL;
	}
		
	// Otherwise attach to the parent:
	else
	{
		if (!(newParentNode->attachmentNode->containsNode(this)))
		{
			newParentNode->attachmentNode->addChild(this);
			newParentNode->as_addChild(this);
		}
	}

	// remove node from current parent:
	if (this->parent != this->newParent) this->detach();
	
	// update the new parent symbols:
	this->parent = this->newParent;
	this->newParent = NULL_SYMBOL;

	// broadcast this change to any remote clients:
	BROADCAST(this, "ss", "setParent", this->parent->s_name);

			
	// update currentNodePath:
    this->updateNodePath();
}

// ***********************************************************
// remove this node from the scenegraph:
void asReferenced::detach()
{
	if (parent == WORLD_SYMBOL)
	{
		if (sceneManager->worldNode->containsNode(this)) sceneManager->worldNode->removeChild(this);
	}

	else {
		osg::ref_ptr<asReferenced> pNode = parent->s_thing;
		if (pNode.valid())
		{
			if (pNode->attachmentNode->containsNode(this))
			{
				pNode->attachmentNode->removeChild(this);
				pNode->as_removeChild(this);
			}
		}
	}

}


// *****************************************************************************
// IMPORTANT:
// subclasses of asReferenced are allowed to contain complicated subgraphs, and
// can also change their attachmentNode so that children are attached anywhere
// in this subgraph. If that is the case, the updateNodePath() function MUST be
// overridden, and extra nodes must be manually pushed onto the currentNodePath.

void asReferenced::updateNodePath()
{
	currentNodePath.clear();
	if ((parent!=WORLD_SYMBOL) && (parent!=NULL_SYMBOL))
	{
		osg::ref_ptr<asReferenced> parentNode = parent->s_thing;
		if (parentNode.valid())
		{
			currentNodePath = parentNode->currentNodePath;
		}
	}
	
	// this nodePath only stores the path until this node (osg::Group).
	currentNodePath.push_back(this);
	
	// now update NodePaths for all children:
	updateChildNodePaths();
	
}


void asReferenced::updateChildNodePaths()
{
	vector<asReferenced*>::iterator childIter;
	for (childIter = children.begin(); childIter != children.end() ; childIter++)
	{
		(*childIter)->updateNodePath();
	}
}


// *****************************************************************************
asReferenced *asReferenced::as_getChild(asReferenced *child)
{
	vector<asReferenced*>::iterator iter;
	for (iter = children.begin(); iter != children.end() ; iter++)
	{
		if ((*iter) == child) return (*iter);
	}
	return NULL;
}

void asReferenced::as_addChild(asReferenced *child)
{
	children.push_back(child);
}

void asReferenced::as_removeChild(asReferenced *child)
{
	vector<asReferenced*>::iterator iter;
	for (iter = children.begin(); iter != children.end() ; iter++)
	{
		if ((*iter) == child)
		{
			children.erase(iter);
			break;
		}
	}
}



// *****************************************************************************
bool asReferenced::legalParent (t_symbol *newParent)
{
	vector<asReferenced*>::iterator childIter;
	
	if (newParent == this->id)
	{
		return false;
	}
	
	else
	{
		for (childIter = children.begin(); childIter != children.end() ; childIter++)
		{
			if ((*childIter)->id == newParent) return false;
		}
	}
	
	return true;
}

// *****************************************************************************

void asReferenced::setParent (const char *newvalue)
{
	t_symbol *s = gensym(newvalue);
	if (parent != s) 
	{
		newParent = s;
	}
}

void asReferenced::setTextFlag (int b)
{
	this->textFlag = (bool) b;
	
	// first remove existing label:
	if (this->attachmentNode->containsNode(textGeode.get()))
	{
		this->attachmentNode->removeChild(textGeode.get());
		textGeode = NULL;
	}
		
	// create a new text label if the labelFlag is set:
	if (this->textFlag)
	{
		
		textGeode = new osg::Geode;
		this->attachmentNode->addChild(textGeode.get());

		osgText::Text *textLabel = new osgText::Text();
		textGeode->addDrawable(textLabel);
				
		// set text:
		textLabel->setText(this->nodeType + "('" + string(this->id->s_name) + "')");

		// set some parameters for the text:
		textLabel->setCharacterSize(0.1f);
		textLabel->setFont(0); // inbuilt font (small)
		//textLabel->setFont(projectPath + "/fonts/arial.ttf");
		textLabel->setFontResolution(40,40);
		textLabel->setAxisAlignment(osgText::Text::SCREEN); // keep facing us
		textLabel->setColor( osg::Vec4(1.0f,1.0f,1.0f,1.0f) );

		//textLabel->setDrawMode(osgText::Text::TEXT);
		//textLabel->setDrawMode(osgText::Text::ALIGNMENT);
		//textLabel->setDrawMode(osgText::Text::BOUNDINGBOX);
		textLabel->setAlignment(osgText::Text::CENTER_BOTTOM); // means text bottom 

		
		// position the text above the bound of the subgraph:
		//const osg::BoundingSphere& bs = this->getBound();
		//float z = bs.center().z()+bs.radius();
		textLabel->setPosition( osg::Vec3(0,0,this->getBound().radius()) );
		
		/*
		// disable lighting effects on the text
		osg::StateSet *labelStateSet = new osg::StateSet;
		//labelStateSet->setMode( GL_LIGHTING, osg::StateAttribute::OFF);
		labelStateSet->setMode( GL_BLEND, osg::StateAttribute::ON );
		labelStateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
		textLabel->setStateSet( labelStateSet );
		*/
	}
	
	
	
/*
	// first remove existing label:
	if (this->containsNode(textTransform.get()))
	{
		this->removeChild(textTransform.get());
		textTransform = NULL;
	}
		
	// create a new text label if the labelFlag is set:
	if (this->textFlag)
	{
		
		// create textTransform and use it to reposition the text
		// outside of the bounding contour of the object (TODO)
		textTransform = new osg::PositionAttitudeTransform();
		textTransform->setName(string(id->s_name) + ".textTransform");
		this->addChild(textTransform.get());

		// create geode and text node:
		osg::Geode *textGeode = new osg::Geode();
		osgText::Text *textLabel = new osgText::Text();
		
		// add them to the scene graph:
		textTransform->addChild(textGeode);
		textGeode->addDrawable(textLabel);
				
		// set text:
		textLabel->setText(this->nodeType + "('" + string(this->id->s_name) + "')");

		//osgText::Font* font = osgText::readFontFile("fonts/arial.ttf");
			
		// set some view parameters for the text:
		textLabel->setCharacterSize(0.25f);
		//textLabel->setFont(projectPath + "/fonts/arial.ttf");
		textLabel->setFontResolution(40,40);
		textLabel->setAxisAlignment(osgText::Text::SCREEN);
		//textLabel->setAxisAlignment(osgText::Text::XZ_PLANE);
		//textLabel->setColor( osg::Vec4(0.0f,0.0f,1.0f,1.0f) );
		textLabel->setColor( osg::Vec4(1.0f,1.0f,1.0f,1.0f) );
		textLabel->setPosition( osg::Vec3(0,0,0.7f) );
		//textLabel->setDrawMode(osgText::Text::TEXT);
		//textLabel->setDrawMode(osgText::Text::ALIGNMENT);
		//textLabel->setDrawMode(osgText::Text::BOUNDINGBOX);
		textLabel->setAlignment(osgText::Text::CENTER_BOTTOM);

	}
	*/
	
	
	
	BROADCAST(this, "si", "setTextFlag", getTextFlag());
}


// *****************************************************************************

void asReferenced::debug()
{
	lo_arg **args;
	int i, argc;
	char *argTypes;
	
	std::cout << "****************************************" << std::endl;
	std::cout << "************* NODE  DEBUG: *************" << std::endl;
	
	std::cout << "\nnode: " << id->s_name << ", type: " << nodeType << std::endl;
	
	vector<lo_message> nodeState = this->getState();
	vector<lo_message>::iterator nodeStateIterator;
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
	

	if (!this->children.empty())
	{
		std::cout << "   children:" << std::endl;
		vector<asReferenced*>::iterator childIter;	
		for (childIter = this->children.begin(); childIter != this->children.end() ; childIter++)
		{
			std::cout << "      " << (*childIter)->id->s_name;
		}
	}
	
}

std::vector<lo_message> asReferenced::getState ()
{
	std::vector<lo_message> ret;
	
	lo_message msg;
	
	msg = lo_message_new();
	lo_message_add(msg, "ss", "setParent", this->getParent());
	ret.push_back(msg);
	
	msg = lo_message_new();
	lo_message_add(msg, "si", "setTextFlag", this->getTextFlag());
	ret.push_back(msg);
	
	return ret;
}
	
// *****************************************************************************
void asReferenced::stateDump ()
{

	vector<lo_message> nodeState = this->getState();

	vector<lo_message>::iterator iter = nodeState.begin();
	while (iter != nodeState.end())
	{
		sceneManager->sendNodeMessage(this->id, (*iter));
		//if (sceneManager->txServ) lo_send_message_from(sceneManager->txAddr, sceneManager->txServ, ("/node/"+string(this->id->s_name)).c_str(), (*iter));
		//lo_message_free(*iter);
		nodeState.erase(iter); //note: iterator automatically advances after erase()
	}
	
}