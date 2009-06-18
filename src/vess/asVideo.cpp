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

#include <osgDB/ReadFile>
#include <osg/Geometry>


#include "asGlobals.h"
#include "osgUtil.h"
#include "asShape.h"
#include "asVideo.h"
#include "asSceneManager.h"
#include "asMediaManager.h"



using namespace std;

//extern asSceneManager *sceneManager;
//extern asMediaManager *mediaManager;


// ===================================================================
// constructor:
asVideo::asVideo (asSceneManager *sceneManager, char *initID) : asReferenced(sceneManager, initID)
{
	this->setName(string(id->s_name) + ".asVideo");
	nodeType = "asVideo";

	useTextureRectangle = false;


	textureName = "NULL";
	renderBin = 11;

	loopMode = 1;
	currentIndex = 0;
	flip = 1;
	playing = 1;

}

// ===================================================================
// destructor
asVideo::~asVideo()
{

}



// ===================================================================
// ======================== SET METHODS: =============================
// ===================================================================


void asVideo::setVideoTexture (char *newTexture)
{
	// don't do anything if the current texture is already loaded:
	if (texturePath == string(newTexture)) return;

	textureName = string(newTexture);
	texturePath = mediaManager->getImagePath(textureName);

	drawVideoTexture();

	BROADCAST(this, "ss", "setVideoTexture", this->getVideoTexture());
}

void asVideo::setRenderBin (int i)
{
	renderBin = i;

	std::cout << "TODO: fix renderbin.  " << renderBin << std::endl;

	//shapeStateSet->setRenderBinDetails( (int)renderBin, "RenderBin");
	//if (shapeGeode.valid()) shapeGeode->setStateSet( shapeStateSet );

	BROADCAST(this, "si", "setRenderBin", renderBin);
}

void asVideo::setFlip (int i)
{
	flip = i;

	if (image.valid())
	{
		image->flipHorizontal();
		image->flipVertical();
	}

	BROADCAST(this, "si", "setFlip", getFlip());
}

void asVideo::setLoopMode (int i)
{
	loopMode = i;

	if (this->imagestream.valid())
	{
		if (loopMode) imagestream->setLoopingMode( osg::ImageStream::LOOPING );
		else imagestream->setLoopingMode( osg::ImageStream::NO_LOOPING );
	}

	BROADCAST(this, "si", "setLoopMode", getLoopMode());
}

void asVideo::setIndex (float f)
{
	// TODO
	std::cout << "Oops. setIndex() is not done yet!" << std::endl;
	BROADCAST(this, "sf", "setIndex", getIndex());
}

void asVideo::play (int i)
{
	playing = i;

	if (this->imagestream.valid())
	{
		if (this->playing) imagestream->play();
		else imagestream->pause();

	}

	BROADCAST(this, "si", "play", getPlay());
}

void asVideo::rewind ()
{
	std::cout << "Rewinding" << std::endl;
	if (this->imagestream.valid())
	{
		imagestream->rewind();
		if (this->playing) imagestream->play();
	}
}




// ===================================================================
void asVideo::drawVideoTexture()
{
	// don't render textures for the master
	if (!sceneManager->isSlave()) return;

	//std::cout << "debug texturepath: " << texturePath <<  std::endl;


	// we need to find the parent (for now, asShape):
	// TODO: apply to any node

	//osg::ref_ptr<osg::Geode> shapeGeode;
	osg::Geode *shapeGeode;


	asReferenced *n = this;
	while (n)
	{
		if (n->nodeType == "asShape")
		{
			asShape *shapeNode = dynamic_cast<asShape*>(n);
			if (shapeNode) shapeGeode = shapeNode->shapeGeode.get();
		}
		n = n->parent->s_thing;
	}

	if (!shapeGeode) return;

	if (texturePath.empty())
	{
		// remove current texture
		// TODO: fix this!
		//shapeGeode->setStateSet( new osg::StateSet() );
		return;
	}

	else
	{



//#if defined(WIN32) || defined(__APPLE__)
        // Force load QuickTime plugin, probe video capability
        osgDB::readImageFile("devices.live");
//#endif


		this->image = osgDB::readImageFile(this->texturePath.c_str());
		if (!image.valid()) return;

		bool autoflip = image->getOrigin()==osg::Image::TOP_LEFT;
		std::cout << "autoflip: " << autoflip << std::endl;

		if (autoflip)
		{
			image->flipHorizontal();
			image->flipVertical();
		}

		this->imagestream = dynamic_cast<osg::ImageStream*>(image.get());

        if (imagestream.valid())
        {
        	this->play(1);
        }

        if (image.valid())
        {

        	osg::StateSet *shapeStateSet = shapeGeode->getOrCreateStateSet();
        	if (!shapeStateSet) return;


        	if (useTextureRectangle)
        	{
        		osg::TextureRectangle* texture = new osg::TextureRectangle(image.get());

            	texture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
            	texture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);

    			// Set Texture:
    			shapeStateSet->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);
        	}

        	else
        	{
        		osg::Texture2D* texture = new osg::Texture2D(image.get());
        		texture->setResizeNonPowerOfTwoHint(false);
        		texture->setFilter(osg::Texture::MIN_FILTER,osg::Texture::LINEAR);

        		//texture->setBorderColor(osg::Vec4(1.0f,1.0f,1.0f,0.0f));
            	texture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
            	texture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);

    			// Set Texture:
    			shapeStateSet->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);
        	}


			// Turn blending on:
			//shapeStateSet->setMode( GL_BLEND, osg::StateAttribute::ON );
			//shapeStateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

			// Disable depth testing so geometry is drawn regardless of depth values
			// of geometry already draw:
			//shapeStateSet->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);

			// Disable lighting:
			shapeStateSet->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

			// Need to make sure this geometry is draw last. RenderBins are handled
			// in numerical order so set bin number to 11
			shapeStateSet->setRenderBinDetails( renderBin, "RenderBin");



			//shapeGeode->setStateSet( shapeStateSet );

        }

        else
        {
        	std::cout << "ERROR (setVideoTexture): The file " << texturePath << " is not a valid texture." << std::endl;
        }

	}

}

std::vector<lo_message> asVideo::getState ()
{
	// inherit state from base class
	std::vector<lo_message> ret = asReferenced::getState();

	lo_message msg;

	msg = lo_message_new();
	lo_message_add(msg,  "ss", "setVideoTexture", getVideoTexture());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "si", "setRenderBin", getRenderBin());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "si", "setFlip", getFlip());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "si", "setLoopMode", getLoopMode());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "sf", "setIndex", getIndex());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "si", "play", getPlay());
	ret.push_back(msg);


	return ret;
}
