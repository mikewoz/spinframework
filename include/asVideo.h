// ===================================================================
// Audioscape library for PureData
// Copyright (c) 2007
//
// Collaborators:
//    Shared Reality Lab (SRE), McGill University Centre for Intelligent Machines (CIM)
//       www.cim.mcgill.ca/sre
//    La Société des Arts Technologiques (SAT)
//       www.sat.qc.ca
//
// Project Directors:
//    Science - Jeremy R. Cooperstock (SRE/CIM)
//    Arts - Zack Settel
//
// Conception:
//    Zack Settel
//
// Development Team:
//    Mike Wozniewski (SRE/CIM): Researcher, Head Developer
//    Zack Settel: Artist, Researcher, Audio/DSP programming
//    Jean-Michel Dumas (SAT): Assistant Researcher
//    Mitchel Benovoy (SRE/CIM): Video Texture Programming
//    Stéphane Pelletier (SRE/CIM): Video Texture Programming
//    Pierre-Olivier Charlebois (SRE/CIM): Former Developer
//
// Funding by / Souventionné par:
//    Natural Sciences and Engineering Research Council of Canada (NSERC)
//    Canada Council for the Arts
//    NSERC/Canada Council for the Arts - New Media Initiative
//
// ===================================================================
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
// ===================================================================


#ifndef __ASTEXTURE_H
#define __ASTEXTURE_H

#include "asGlobals.h"
#include "asReferenced.h"

#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osg/Texture2D>
#include <osg/ImageStream>
#include <osg/TextureRectangle>
#include <osgUtil/Optimizer>

/**
 * \brief A video texture node that allows dynamic textures to be applied to its
 *        parent (which must be an asShape for now)
 * 
 * This uses osg::ImageStream, so it can load several different movie formats,
 * including live feeds on the Mac via devices.live.
 */
class asVideo : public asReferenced
{

public:

	asVideo(asSceneManager *sceneManager, char *initID);
	virtual ~asVideo();

	void setVideoTexture   (char* newTexture);
	void setRenderBin (int i);

	void setFlip (int i);
	void setLoopMode (int i);
	void setIndex (float f);
	void play (int i);
	void rewind ();


	const char *getVideoTexture() { return textureName.c_str(); }
	int getRenderBin() { return renderBin; }

	int getFlip() { return flip; }
	int getLoopMode() { return loopMode; }
	float getIndex() { return currentIndex; }
	int getPlay() { return playing; }

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



	void drawVideoTexture();


	bool useTextureRectangle;


	std::string texturePath;
	std::string textureName;


	int renderBin;

	int flip;
	int loopMode;
	float currentIndex;
	int playing;



	osg::ref_ptr<osg::Image> image;
	osg::ref_ptr<osg::ImageStream> imagestream;
	//osg::observer_ptr<osg::ImageStream> imagestream;



private:


};


#endif
