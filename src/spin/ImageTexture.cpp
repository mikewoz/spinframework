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
#include <osg/TextureRectangle>
#include <osg/Texture2D>
#include <osgDB/ReadFile>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>

#include <iostream>

#include "SceneManager.h"
#include "ImageTexture.h"
#include "spinApp.h"
#include "spinBaseContext.h"

namespace spin
{

// *****************************************************************************
// constructor:
ImageTexture::ImageTexture (SceneManager *s, const char *initID) : ReferencedStateSet(s, initID)
{
	classType = "ImageTexture";

	_path = "NULL";
	_renderBin = 11;
	_lightingEnabled = true;
	_useTextureRectangle = false;
}

// destructor
ImageTexture::~ImageTexture()
{
}

// *****************************************************************************

void ImageTexture::debug()
{
	ReferencedStateSet::debug();
	
	std::cout << "   ---------" << std::endl;

	// additional info
	if (_image.valid())
	{
		std::cout << "   Transparency? " << _image->isImageTranslucent() << std::endl;
	}
}


// *****************************************************************************
void ImageTexture::setPath (const char* newPath)
{
	// only do this if the id has changed:
	if (_path == std::string(newPath)) return;

	_path = std::string(newPath);
	
	if (sceneManager->isGraphical())
	{

		std::string fullPath = getAbsolutePath(_path);

		std::cout << "Loading image: " << fullPath << std::endl;

		osg::ref_ptr<osg::Image> test = osgDB::readImageFile(fullPath);
		
		if (test.valid())
		{
			this->setName("ImageTexture("+_path+")");

			// create a texture object
			osg::Texture *tex;
			if (_useTextureRectangle)
			{
				tex = new osg::TextureRectangle;
			} else {
				tex = new osg::Texture2D;
			}
		
			tex->setResizeNonPowerOfTwoHint(false);
			tex->setFilter(osg::Texture::MIN_FILTER,osg::Texture::LINEAR);
			//tex->setFilter(osg::Texture::MAG_FILTER,osg::Texture::LINEAR);
			//tex->setWrap(osg::Texture::WRAP_R,osg::Texture::REPEAT);
			tex->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
			tex->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);

			// set the image:
			tex->setImage(0,test.get());
			
			// set transparent border:
			tex->setBorderColor(osg::Vec4(1.0f,1.0f,1.0f,0.0f));
		
			// add texture to stateset:
			this->setTextureAttributeAndModes(0, tex, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE );

			// set lighting:
			if (_lightingEnabled) this->setMode( GL_LIGHTING, osg::StateAttribute::ON );
			else this->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

			// set renderbin:
			this->setRenderBinDetails( _renderBin, "RenderBin");

			// if image has transparency, enable blending:
			if (1)//(_imageStream->isImageTranslucent())
			{
				this->setMode(GL_BLEND, osg::StateAttribute::ON);
				this->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
			}
		
		}

		else {
			std::cout << "ImageTexture ERROR. Bad format?: " << _path << std::endl;
		}


	}

	BROADCAST(this, "ss", "setPath", getPath());
}

void ImageTexture::setLighting (int i)
{
	_lightingEnabled = (bool)i;

	if (_lightingEnabled) this->setMode( GL_LIGHTING, osg::StateAttribute::ON );
	else this->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

	BROADCAST(this, "si", "setLighting", getLighting());
}

void ImageTexture::setRenderBin (int i)
{
	_renderBin = i;
	this->setRenderBinDetails( (int)_renderBin, "RenderBin");

	BROADCAST(this, "si", "setRenderBin", getRenderBin());
}

// *****************************************************************************
std::vector<lo_message> ImageTexture::getState () const
{
	// inherit state from base class
	std::vector<lo_message> ret = ReferencedStateSet::getState();
		
	lo_message msg;
	
	msg = lo_message_new();
	lo_message_add(msg, "ss", "setPath", getPath());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "si", "setLighting", getLighting());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "si", "setRenderBin", getRenderBin());
	ret.push_back(msg);
	
	return ret;
}

bool ImageTexture::isValid() const { return (_image.valid()); }

} // end of namespace spin

