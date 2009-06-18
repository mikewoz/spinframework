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

#ifndef ASLIGHTSOURCE_H_
#define ASLIGHTSOURCE_H_



#include "asGlobals.h"
#include "asReferenced.h"

#include <osg/Light>
#include <osg/LightSource>

/**
 * \brief A light source, with controllable intensity, color, etc.
 *
 * Note that only 8 light sources are available.
 */
class asLightSource : public asReferenced
{

public:

	asLightSource(asSceneManager *sceneManager, char *initID);
	virtual ~asLightSource();

	void setVisible		(int b);
	void setCutoff		(float cut);
	void setExponent	(float exp);
	void setAttenuation	(float att);
	
	void setAmbient		(float r, float g, float b, float a);
	void setDiffuse		(float r, float g, float b, float a);
	void setSpecular	(float r, float g, float b, float a);
	
	int getVisible() 		{ return (int) this->_visible; }
	float getCutoff()		{ return this->_cutoff; };
	float getExponent()		{ return this->_exponent; };
	float getAttenuation()	{ return this->_attenuation; };
	osg::Vec4 getAmbient()	{ return this->_ambient; };
	osg::Vec4 getDiffuse()	{ return this->_diffuse; };
	osg::Vec4 getSpecular()	{ return this->_specular; };
	
	
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
	
	void drawLight();	
	
	int lightNum;
	osg::ref_ptr<osg::LightSource> lightSource;
	
	bool _visible;
	float _cutoff;
	float _exponent;
	float _attenuation;
	
	// lighting color parameters:
	osg::Vec4 _ambient;
	osg::Vec4 _diffuse;
	osg::Vec4 _specular;
	
};



#endif
