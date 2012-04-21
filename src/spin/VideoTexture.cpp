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
#include <osg/ImageStream>
#include <osg/ImageSequence>
#include <osg/TextureRectangle>
#include <osg/Texture2D>
#include <osg/StateAttribute>
#include <osgDB/ReadFile>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>

#include <iostream>
#include "spinApp.h"
#include "SceneManager.h"
#include "spinBaseContext.h"
#include "VideoTexture.h"

namespace spin
{

// *****************************************************************************
// constructor:
VideoTexture::VideoTexture (SceneManager *s, const char *initID) : Shader(s, initID)
{
	classType = "VideoTexture";

	_useTextureRectangle = false;
	lightingEnabled_ = false;
	
	_index = 0.0;
	_loop = true;
	_play = true;
	_framerate = 24;

	/*
	// force removal of xine plugin (encourage ffmpeg):
	osgDB::ReaderWriter *rw = osgDB::Registry::instance()->getReaderWriterForExtension("xine");
	if (rw) osgDB::Registry::instance()->removeReaderWriter(rw);

	std::string xineLib = osgDB::Registry::instance()->createLibraryNameForExtension("xine");
	std::cout << "closing xineLib: " << xineLib << std::endl;
	std::cout << osgDB::Registry::instance()->closeLibrary(xineLib) << std::endl;
	*/
}

// destructor
VideoTexture::~VideoTexture()
{
	if (_imageStream.valid()) _imageStream->quit();
}

// *****************************************************************************

void VideoTexture::debug()
{
	Shader::debug();
	
	std::cout << "   ---------" << std::endl;
	
	osg::ref_ptr<osg::ImageSequence> tmpImageSequence = dynamic_cast<osg::ImageSequence*>(_imageStream.get());

	
	if (tmpImageSequence.valid())
	{
		std::cout << "   Type: Image sequence" << std::endl;
		std::cout << "   NumFrames: " << tmpImageSequence->getNumImages() << std::endl;
	}
	else if (_imageStream.valid())
	{
		std::cout << "   Type: Video file" << std::endl;
	} else {
		std::cout << "   Type: NOT VALID" << std::endl;
	}
	
	// additional info
	if (_imageStream.valid())
	{
		std::cout << "   Duration: " << _imageStream->getLength() << "s" << std::endl;
		std::cout << "   TimeMultiplier: " << _imageStream->getTimeMultiplier() << "s" << std::endl;
		std::cout << "   Status: ";
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
		std::cout << "   Transparency? " << _imageStream->isImageTranslucent() << std::endl;
	}
}


// *****************************************************************************
void VideoTexture::setPath (const char* newPath)
{
	// only do this if the id has changed:
	if (_path == std::string(newPath)) return;

	_path = std::string(newPath);

	//debug
	//osg::setNotifyLevel(osg::DEBUG_FP);
	
	if (sceneManager->isGraphical())
	{
		osg::ref_ptr<osg::ImageStream> test;
		
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
		//vidTexture->setFilter(osg::Texture::MAG_FILTER,osg::Texture::LINEAR);
		if (textureRepeatS_)
			vidTexture->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
		else
			vidTexture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP);
		if (textureRepeatT_)
			vidTexture->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
		else
			vidTexture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP);

		osg::TexEnv* texenv = new osg::TexEnv();
		texenv->setMode(textureBlend_);
		this->setTextureAttribute(0, texenv, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE );

		
		// Check if _path is a directory. If so, load all contained image
		// files into an osg::ImageSequence.
		osgDB::DirectoryContents dir = osgDB::getDirectoryContents(fullPath);
	
		if (dir.size())
		{
			std::cout << "Creating VideoTexture from image sequence";
			osg::ref_ptr<osg::ImageSequence> tmpImageSequence = new osg::ImageSequence();

			// sort directory contents:
			std::sort(dir.begin(), dir.end());

			// search for image files
			for (osgDB::DirectoryContents::iterator itr = dir.begin(); itr != dir.end(); ++itr)
			{
				// ignore filenames that start with a .
				if ((*itr)[0] != '.')
				{
					std::cout << "."; fflush(stdout);
				
					//tmpImageSequence->addImageFile(osgDB::concatPaths(fullPath,*itr));
					osg::ref_ptr<osg::Image> image = osgDB::readImageFile(osgDB::concatPaths(fullPath,*itr));
					if (image.valid())
					{
						tmpImageSequence->addImage(image.get());
					} else {
						std::cout << std::endl << "invalid image: " << (*itr) << std::endl;
					}
				}
			}
			std::cout << std::endl;
		
			if (tmpImageSequence->getNumImages())
			{
				test = tmpImageSequence.get();
				
				vidTexture->setImage(0,tmpImageSequence.get());
			
				setFrameRate(_framerate);

				this->_isSequence = true;	
			}
		
			else {
				std::cout << "Oops. The specified folder did not contain an image sequence: " << _path << std::endl;
				tmpImageSequence.release();
			}
		
		}
	
		// Otherwise, assume it is a file and use osgDB::readImageFile to let
		// osgPlugins try to load it
		else {

			// preload ffmpeg plugin
			std::string libName = osgDB::Registry::instance()->createLibraryNameForExtension("ffmpeg");
			osgDB::Registry::instance()->loadLibrary(libName);

			osg::ref_ptr<osg::Image> image = osgDB::readImageFile(fullPath);
			test = dynamic_cast<osg::ImageStream*>(image.get());
		
			if (test.valid())
			{
				this->setName("VideoTexture("+_path+")");			
				vidTexture->setImage(0,test.get());
				test->rewind();
				std::cout << "loaded video " << _path << ". Duration: " << test->getLength() << std::endl;
			}
			else {	
				std::cout << "VideoTexture ERROR. Not a video format?: " << _path << std::endl;
			}	
		}
	
		// Once we have the imageStream, add it to the StateSet
		if (test.valid())
		{
			this->_imageStream = test.get();

			// add texture to stateset:
			this->setTextureAttributeAndModes( 0, vidTexture, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE );

			// set lighting:
			if (lightingEnabled_) this->setMode( GL_LIGHTING, osg::StateAttribute::ON );
			else this->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

			// set renderbin:
			this->setRenderBinDetails( renderBin_, "RenderBin");

			// if image has transparency, enable blending:
			if (1)//(_imageStream->isImageTranslucent())
			{
				this->setMode(GL_BLEND, osg::StateAttribute::ON);
				this->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
			}
		
			// apply current play/loop state, overriding OSG defaults:
			if (_play) _imageStream->play();
			else _imageStream->pause();
			if (!_loop) _imageStream->setLoopingMode( osg::ImageStream::NO_LOOPING );
		}

	}

	//osg::setNotifyLevel(osg::FATAL);

	BROADCAST(this, "ss", "setPath", getPath());
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
		
		BROADCAST(this, "si", "setLoop", getLoop());
	}
}

void VideoTexture::setIndex (float f)
{
	_index = f;
	
	if (_imageStream.valid())
	{
		double seekIndex = (double) (f * _imageStream->getLength());
		std::cout << "seeking to " << seekIndex << std::endl;
		_imageStream->seek(seekIndex);
	}
	
	BROADCAST(this, "sf", "setIndex", getIndex());
}

void VideoTexture::setFrameRate (float f)
{	
	osg::ref_ptr<osg::ImageSequence> tmpImageSequence = dynamic_cast<osg::ImageSequence*>(_imageStream.get());
	
	if (tmpImageSequence.valid())
	{
		tmpImageSequence->setLength(tmpImageSequence->getNumImages() / f);
	}
}

void VideoTexture::setPlay (int i)
{
	if (_play != (int)i)
	{
		_play = (bool)i;
		
		if (_imageStream.valid())
		{
			if (_play) _imageStream->play();
			else _imageStream->pause();
		}
		
		BROADCAST(this, "si", "setPlay", getPlay());
	}
}

void VideoTexture::rewind ()
{
	if (_imageStream.valid())
	{
		_imageStream->rewind();
		if (_play) _imageStream->play();
	}
	
	BROADCAST(this, "s", "rewind");
}

void VideoTexture::flipHorizontal()
{
	if (_imageStream.valid())
	{
		_imageStream->flipHorizontal();
	}
	BROADCAST(this, "s", "flipHorizontal");
}

void VideoTexture::flipVertical()
{
	if (_imageStream.valid())
	{
		_imageStream->flipVertical();
	}
	BROADCAST(this, "s", "flipVertical");
}



// *****************************************************************************
std::vector<lo_message> VideoTexture::getState () const
{
	// inherit state from base class
	std::vector<lo_message> ret = Shader::getState();
		
	lo_message msg;
	
	msg = lo_message_new();
	lo_message_add(msg, "ss", "setPath", getPath());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "si", "setLoop", getLoop());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "sf", "setIndex", getIndex());
	ret.push_back(msg);
	
	msg = lo_message_new();
	lo_message_add(msg, "sf", "setFrameRate", getFrameRate());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "si", "setPlay", getPlay());
	ret.push_back(msg);

	return ret;
}

} // end of namespace spin

