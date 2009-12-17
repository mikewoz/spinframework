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
#include <osgDB/ReadFile>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>

#include <iostream>

#include "SceneManager.h"
#include "VideoTexture.h"



using namespace std;



// *****************************************************************************
// constructor:
//VideoTexture::VideoTexture (const char *initID) : osg::TextureRectangle()
VideoTexture::VideoTexture (const char *initID) : osg::StateSet()
{
	_id = gensym(initID);
	
	_useTextureRectangle = false;
	
	
}

// destructor
VideoTexture::~VideoTexture()
{
	if (_imageStream.valid()) _imageStream->quit();
	this->clear();
}

// *****************************************************************************


/*
void VideoTexture::updateCallback()
{

    // do update here
    
}
*/

void VideoTexture::debugPrint ()
{	
	std::cout << "VideoTexture (" << _id->s_name << "):" << std::endl;
	
	osg::ref_ptr<osg::ImageSequence> _imageSequence = dynamic_cast<osg::ImageSequence*>(_imageStream.get());

	if (_imageSequence.valid())
	{
		std::cout << "  Type:\t\tImage sequence" << std::endl;
		std::cout << "  NumFrames:\t" << _imageSequence->getNumImages() << std::endl;
	}
	else if (_imageStream.valid())
	{
		std::cout << "  Type:\t\tVideo file" << std::endl;
	} else {
		std::cout << "  NOT VALID" << std::endl;
	}
	
	// additional info
	if (_imageStream.valid())
	{
		std::cout << "  Duration:\t" << _imageStream->getLength() << "s" << std::endl;
		std::cout << "  Status:\t";
		switch (_imageStream->getStatus())
		{
		case osg::ImageStream::INVALID:
			std::cout << "INVALID" << std::endl; break;
		case osg::ImageStream::PLAYING:
			std::cout << "PLAYING" << std::endl; break;
		case osg::ImageStream::PAUSED:
			std::cout << "PAUSED" << std::endl; break;
		case osg::ImageStream::REWINDING:
			std::cout << "REWINDING" << std::endl; break;
		}
		std::cout << "  Looping?\t" << _imageStream->getLoopingMode() << std::endl;
		std::cout << "  Transparency?\t" << _imageStream->isImageTranslucent() << std::endl;
	}
	
	else {
	}
	
	std::cout << "  Path:\t\t" << _path << std::endl;
	
}


// *****************************************************************************
void VideoTexture::setVideoPath (const char* newPath)
{
	// only do this if the id has changed:
	if (_path == std::string(newPath)) return;

	_path = std::string(newPath);

	std::string fullPath = getAbsolutePath(_path);
	
	
	// create a texture object
	osg::Texture *vidTexture;
	if (_useTextureRectangle)
	{
		vidTexture = new osg::TextureRectangle;
	} else {
		vidTexture = new osg::Texture2D;
	}
	
	vidTexture->setResizeNonPowerOfTwoHint(false);
	vidTexture->setFilter(osg::Texture::MIN_FILTER,osg::Texture::LINEAR);
	vidTexture->setFilter(osg::Texture::MAG_FILTER,osg::Texture::LINEAR);
	//vidTexture->setWrap(osg::Texture::WRAP_R,osg::Texture::REPEAT);
	vidTexture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
	vidTexture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);

	
	// Check if _path is a directory. If so, load all contained image
	// files into an osg::ImageSequence.
	osgDB::DirectoryContents dir = osgDB::getDirectoryContents(fullPath);
	
	if (dir.size())
	{
		std::cout << "Creating VideoTexture from image sequence";

		osg::ref_ptr<osg::ImageSequence> _imageSequence = new osg::ImageSequence();

		// search for image files
		for (osgDB::DirectoryContents::iterator itr = dir.begin(); itr != dir.end(); ++itr)
		{
			// ignore filenames that start with a .
			if ((*itr)[0] != '.')
			{
				std::cout << "."; fflush(stdout);
				
				//_imageSequence->addImageFile(osgDB::concatPaths(fullPath,*itr));
				osg::ref_ptr<osg::Image> image = osgDB::readImageFile(osgDB::concatPaths(fullPath,*itr));
				if (image.valid())
				{
					_imageSequence->addImage(image.get());
				} else {
					std::cout << std::endl << "invalid image: " << (*itr) << std::endl;
				}
			}
		}
		std::cout << std::endl;
		
		if (_imageSequence->getNumImages())
		{		
			vidTexture->setImage(0,_imageSequence.get());
			
			this->_imageStream = _imageSequence.get();
			
			// assume 24fps as a default;
			setFrameRate(24);

			this->_isSequence = true;	
		}
		
		else {
			std::cout << "Oops. The specified folder did not contain an image sequence: " << _path << std::endl;
			_imageSequence.release();
		}
		
	}
	
	// Otherwise, assume it is a file and use osgDB::readImageFile to let
	// osgPlugins try to load it
	else {

		osg::ref_ptr<osg::Image> image = osgDB::readImageFile(fullPath);
		_imageStream = dynamic_cast<osg::ImageStream*>(image.get());
		
		if (_imageStream.valid())
		{
			this->setName("VideoTexture("+_path+")");			
			vidTexture->setImage(0,_imageStream.get());
		}
		else {	
			std::cout << "VideoTexture ERROR. Not a video format?: " << _path << std::endl;
		}	
	}
	
	// Once we have the imageStream, add it to the StateSet
	if (_imageStream.valid())
	{
		// add texture to stateset:
		this->setTextureAttributeAndModes(0, vidTexture, osg::StateAttribute::ON);

		// turn off lighting 
		this->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

		// if image has transparency, enable blending:
		if (_imageStream->isImageTranslucent())
		{
			this->setMode(GL_BLEND, osg::StateAttribute::ON);
			this->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
		}	
	}
	
}

void VideoTexture::setFlip (int i)
{
	if (_flip != (int)i)
	{
		_flip = i;
		_imageStream->flipHorizontal();
		//BROADCAST(this, "si", "setFlip", getFlip());
	}
}

void VideoTexture::setLoop (int i)
{
	if (_loop != (int)i)
	{
		_loop = i;

		if (_imageStream.valid())
		{
			if (_loop) _imageStream->setLoopingMode( osg::ImageStream::LOOPING );
			else _imageStream->setLoopingMode( osg::ImageStream::NO_LOOPING );
		}
		
		//BROADCAST(this, "si", "setLoop", getLoop());
	}
}

void VideoTexture::setIndex (float f)
{
	_index = f;
	
	if (_imageStream.valid())
	{
		_imageStream->seek(f);
	}
	
	//BROADCAST(this, "sf", "setIndex", getIndex());
}

void VideoTexture::setFrameRate (float f)
{	
	osg::ref_ptr<osg::ImageSequence> _imageSequence = dynamic_cast<osg::ImageSequence*>(_imageStream.get());
	
	if (_imageSequence.valid())
	{
		_imageSequence->setLength(_imageSequence->getNumImages() / f);
	}
}

void VideoTexture::setPlay (int i)
{
	if (_play != (int)i)
	{
		_play = i;
		
		if (_imageStream.valid())
		{
			if (_play) _imageStream->play();
			else _imageStream->pause();
		}
		
		/*
		if (_imageSequence.valid())
		{
			if (_play) _imageSequence->play();
			else _imageSequence->pause();
		}
		*/
		
		//BROADCAST(this, "si", "setPlay", getPlay());
	}
}

void VideoTexture::rewind ()
{
	std::cout << "Rewinding" << std::endl;
	if (_imageStream.valid())
	{
		_imageStream->rewind();
		if (_play) _imageStream->play();
	}
}

