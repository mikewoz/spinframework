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
#include "ShaderUtil.h"

extern pthread_mutex_t sceneMutex;

namespace spin
{

// *****************************************************************************
// constructor:
ReferencedStateSet::ReferencedStateSet(SceneManager *s, const char *initID)
{

	id_ = gensym(initID);
	id_->s_thing = this;
	id_->s_type = REFERENCED_STATESET;

	renderBin_ = 11;
	lightingEnabled_ = true;
    transparent_ = false;
	textureBlend_ = osg::TexEnv::MODULATE;
	textureRepeatS_ = false;
	textureRepeatT_ = false;
	
	sceneManager_ = s;
	
	classType_ = "ReferencedStateSet";
	
	this->setName(std::string(id_->s_name) + ".ReferencedStateSet");
	
	// We need to set up a callback. This should be on the topmost node, so that during node
	// traversal, we update our parameters before anything is drawn.
	this->setUserData( dynamic_cast<osg::Referenced*>(this) );
	this->setUpdateCallback(new ReferencedStateSet_callback);
    
    // initialize some stuff:
    /*
    osg::TexEnv* texenv = new osg::TexEnv();
    texenv->setMode(textureBlend_);
	this->setTextureAttribute(0, texenv, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE );
    */
	
    // set lighting:
    if (lightingEnabled_) this->setMode( GL_LIGHTING, osg::StateAttribute::ON );
	else this->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

	// set renderbin:
	this->setRenderBinDetails( renderBin_, "RenderBin");

	// if image has transparency, enable blending:
	if (0)//(_imageStream->isImageTranslucent())
	{
		this->setMode(GL_BLEND, osg::StateAttribute::ON);
		this->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	}
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

	std::cout << "\nReferencedStateSet: " << id_->s_name << ", type: " << classType_ << std::endl;

	
	std::cout << "   Shared by:";
	for (unsigned i = 0; i < getNumParents(); i++)
		std::cout << " " << getParent(i)->getName();
	std::cout << std::endl;

	//osg::ref_ptr<ReferencedStateSet> test = this;
	//std::cout << "ref_count=" << test->getReferenceCount() << std::endl;
	
	
	std::vector<lo_message> nodeState = this->getState();
	std::vector<lo_message>::iterator nodeStateIterator;
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

void ReferencedStateSet::setTextureBlend (int mode)
{
	switch (mode)
	{
		case 1:
			textureBlend_ = osg::TexEnv::DECAL;
			break;
		case 2:
			textureBlend_ = osg::TexEnv::BLEND;
			break;
		case 3:
			textureBlend_ = osg::TexEnv::REPLACE;
			break;
		default:
			textureBlend_ = osg::TexEnv::MODULATE;
			break;
	}
	
	osg::StateSet::TextureAttributeList& texList = this->getTextureAttributeList();
	for (unsigned int i=0; i<texList.size(); ++i)
	{
		osg::TexEnv* texenv = dynamic_cast<osg::TexEnv*>(this->getTextureAttribute(i,osg::StateAttribute::TEXENV));
		if (texenv)
		{
			texenv->setMode(textureBlend_);
		}
	}

	BROADCAST(this, "si", "setTextureBlend", getTextureBlend());
}

int ReferencedStateSet::getTextureBlend() const
{
	switch (textureBlend_)
	{
		case osg::TexEnv::DECAL:
			return 1;
		case osg::TexEnv::BLEND:
			return 2;
		case osg::TexEnv::REPLACE:
			return 3;
		default:
			return 0;
	}
}	

void ReferencedStateSet::setTextureRepeat (int s, int t)
{
	textureRepeatS_=(bool)s;
	textureRepeatT_=(bool)t;
	
	osg::StateSet::TextureAttributeList& texList = this->getTextureAttributeList();
	for (unsigned int i=0; i<texList.size(); ++i)
	{
		osg::Texture* tex = dynamic_cast<osg::Texture*>(this->getTextureAttribute(i,osg::StateAttribute::TEXTURE));
		if (tex)
		{
			// TODO: what is WRAP_R ?
			//tex->setWrap(osg::Texture::WRAP_R, osg::Texture::REPEAT);
			if (textureRepeatS_)
				tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
			else
				tex->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP);
			if (textureRepeatT_)
				tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
			else
				tex->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP);
		}
	}
	
	BROADCAST(this, "sii", "setTextureRepeat", (int)textureRepeatS_, (int)textureRepeatT_);
}

void ReferencedStateSet::setLighting (int i)
{
	lightingEnabled_ = (bool)i;

	if (lightingEnabled_) this->setMode( GL_LIGHTING, osg::StateAttribute::ON );
	else this->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

	BROADCAST(this, "si", "setLighting", getLighting());
}

void ReferencedStateSet::setTransparent (int i)
{
	transparent_ = (bool)i;

	if (transparent_) this->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	else this->setRenderingHint(osg::StateSet::DEFAULT_BIN);

	BROADCAST(this, "si", "setTransparent", getTransparent());
}

void ReferencedStateSet::setRenderBin (int i)
{
	renderBin_ = i;
	this->setRenderBinDetails( (int)renderBin_, "RenderBin");

	BROADCAST(this, "si", "setRenderBin", getRenderBin());
}


// *****************************************************************************
std::vector<lo_message> ReferencedStateSet::getState () const
{
	std::vector<lo_message> ret;

	lo_message msg;

	msg = lo_message_new();
	lo_message_add(msg, "si", "setTextureBlend", getTextureBlend());
	ret.push_back(msg);
	
	msg = lo_message_new();
	lo_message_add(msg, "sii", "setTextureRepeat", (int)textureRepeatS_, (int)textureRepeatT_);
	ret.push_back(msg);
	
	msg = lo_message_new();
	lo_message_add(msg, "si", "setLighting", getLighting());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "si", "setTransparent", getTransparent());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "si", "setRenderBin", getRenderBin());
	ret.push_back(msg);


	
	return ret;
}


void ReferencedStateSet::stateDump()
{
    spinApp::Instance().NodeBundle(this->getID(), this->getState());
}


void ReferencedStateSet::stateDump(lo_address addr)
{
    spinApp::Instance().NodeBundle(this->getID(), this->getState(), addr);
}

} // end of namespace spin

