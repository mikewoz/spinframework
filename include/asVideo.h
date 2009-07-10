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

#ifndef __ASVIDEO_H
#define __ASVIDEO_H

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
	//virtual void stateDump() { asReferenced::stateDump(); };



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
