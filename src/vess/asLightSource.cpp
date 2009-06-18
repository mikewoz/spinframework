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

#include "asLightSource.h"
#include "asSceneManager.h"

#include <string>
#include <iostream>

using namespace std;

//extern asSceneManager *sceneManager;


// *****************************************************************************
// constructor:
asLightSource::asLightSource (asSceneManager *sceneManager, char *initID) : asReferenced(sceneManager, initID)
{
	this->setName(string(id->s_name) + ".asLightSource");
	nodeType = "asLightSource";
	
	_visible = true;
	
	_cutoff = 25.0f;
	_exponent = 25.0f;
	_attenuation = 0.1f;
	
	_ambient = osg::Vec4(0.5,0.5,0.5,1);
	_diffuse = osg::Vec4(0.5,0.5,0.5,1);
	_specular = osg::Vec4(0.5,0.5,0.5,1);
	
	drawLight();
}

// *****************************************************************************
// destructor
asLightSource::~asLightSource()
{
	std::cout << "destroying lightsource: " << this->id->s_name << std::endl;
	
	osg::Light *light;
	osg::StateSet* thisStateSet = sceneManager->rootNode->getOrCreateStateSet();
	if ( this->containsNode( lightSource.get() )) // kill light
	{
		light = lightSource->getLight();
		sceneManager->activeLights[light->getLightNum()] = false;
		
		if (thisStateSet)
		{
			lightSource->setLocalStateSetModes(osg::StateAttribute::OFF);
			lightSource->setStateSetModes(*thisStateSet, osg::StateAttribute::OFF);
		}
	
		this->removeChild( lightSource.get() );
		lightSource = NULL;
		lightNum = -1;
	}
}


// *****************************************************************************
void asLightSource::setVisible (int b)
{
	if (this->_visible != (bool)b)
	{
		this->_visible = (bool) b;
		drawLight();
		BROADCAST(this, "si", "setVisible", (int) this->_visible);	
	}
}

void asLightSource::setCutoff (float cut)
{
	if (this->_cutoff != cut)
	{
		this->_cutoff = cut;	
		drawLight();
		BROADCAST(this, "sf", "setCutoff", this->_cutoff);
	}
}

void asLightSource::setExponent (float exp)
{
	if (this->_exponent != exp)
	{
		this->_exponent = exp;
		drawLight();	
		BROADCAST(this, "sf", "setExponent", this->_exponent);
	}
}

void asLightSource::setAttenuation (float att)
{
	if (this->_attenuation != att)
	{
		this->_attenuation = att;
		drawLight();	
		BROADCAST(this, "sf", "setAttenuation", this->_attenuation);
	}
}

void asLightSource::setAmbient (float r, float g, float b, float a)
{
	if (this->_ambient != osg::Vec4(r,g,b,a))
	{
		this->_ambient = osg::Vec4(r,g,b,a);
		drawLight();
		BROADCAST(this, "sffff", "setAmbient", r, g, b, a);
	}
}

void asLightSource::setDiffuse (float r, float g, float b, float a)
{
	if (this->_diffuse != osg::Vec4(r,g,b,a))
	{
		this->_diffuse = osg::Vec4(r,g,b,a);
		drawLight();
		BROADCAST(this, "sffff", "setDiffuse", r, g, b, a);
	}
}

void asLightSource::setSpecular (float r, float g, float b, float a)
{
	if (this->_specular != osg::Vec4(r,g,b,a))
	{
		this->_specular = osg::Vec4(r,g,b,a);
		drawLight();
		BROADCAST(this, "sffff", "setSpecular", r, g, b, a);
	}
}

// *****************************************************************************
void asLightSource::drawLight()
{
	if (!sceneManager->isGraphical) return;

	osg::Light *light;
	osg::StateSet* thisStateSet = sceneManager->rootNode->getOrCreateStateSet();
	
	if (!this->_visible && this->containsNode( lightSource.get() ))
	{
		// kill light:
		
		light = lightSource->getLight();
		sceneManager->activeLights[light->getLightNum()] = false;
		
		//std::cout << "killing light num " << light->getLightNum() << std::endl;
		
		if (thisStateSet)
		{
			lightSource->setLocalStateSetModes(osg::StateAttribute::OFF);
			lightSource->setStateSetModes(*thisStateSet, osg::StateAttribute::OFF);
		}
	
        this->removeChild( lightSource.get() );
		lightSource = NULL;
		lightNum = -1;
		
	}
	 
    else if (this->_visible && !this->containsNode(lightSource.get()) )
	{
    	// instantiate light !!

		// find next free spot
		for (int i=0; i<OSG_NUM_LIGHTS; i++)
			if (!sceneManager->activeLights[i]) {lightNum = i; break;}

		//std::cout << "... activating light. found available lightNum: " << lightNum << std::endl;
		
		if (lightNum>=0)
		{
			// create a spot light with standard parameters
			lightSource = new osg::LightSource();

            this->addChild( lightSource.get() );

                
			light = lightSource->getLight();
		
			light->setLightNum(lightNum);
			sceneManager->activeLights[lightNum] = true;

			if (thisStateSet)
			{
				lightSource->setLocalStateSetModes(osg::StateAttribute::ON);
				lightSource->setStateSetModes(*thisStateSet, osg::StateAttribute::ON);
			}

			light->setPosition(osg::Vec4(0,0,0, 1.0f));
			light->setDirection(osg::Vec3(0.0,1.0,0.0));
			
		} else {
			std::cout << "ERROR: tried to activate lightbeam for " << this->id->s_name << ", but all " << OSG_NUM_LIGHTS << " lights are already in use." << std::endl;
		}
		
	}

	if (lightSource.valid())
	{
		light = lightSource->getLight();

		//std::cout << "... setting light params for light number: " << light->getLightNum() << std::endl;
		
		light->setSpecular( _specular );
		light->setAmbient( _ambient );
		light->setDiffuse( _diffuse );
	    
		light->setSpotCutoff(_cutoff);
	    light->setSpotExponent(_exponent);
	    
		//(length > 70) ? attenuation = 0 : attenuation = 0.1 - length / 700.0;
	    
	    light->setConstantAttenuation(1.0f);
	    light->setLinearAttenuation(_attenuation);
	
	}
}


// *****************************************************************************

std::vector<lo_message> asLightSource::getState ()
{
	// inherit state from base class
	std::vector<lo_message> ret = asReferenced::getState();
	
	lo_message msg;
	osg::Vec4 v;
	
	msg = lo_message_new();
	lo_message_add(msg, "si", "setVisible", (int)_visible);
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "sf", "setCutoff", _cutoff);
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "sf", "setExponent", _exponent);
	ret.push_back(msg);
	
	msg = lo_message_new();
	lo_message_add(msg, "sf", "setAttenuation", _attenuation);
	ret.push_back(msg);

	msg = lo_message_new();
	v = this->getAmbient();
	lo_message_add(msg, "sffff", "setAmbient", v.x(), v.y(), v.z(), v.w());
	ret.push_back(msg);
	
	msg = lo_message_new();
	v = this->getDiffuse();
	lo_message_add(msg, "sffff", "setDiffuse", v.x(), v.y(), v.z(), v.w());
	ret.push_back(msg);
	
	msg = lo_message_new();
	v = this->getSpecular();
	lo_message_add(msg, "sffff", "setSpecular", v.x(), v.y(), v.z(), v.w());
	ret.push_back(msg);
	
	
	return ret;
}