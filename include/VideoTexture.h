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

#ifndef __VideoTexture_H
#define __VideoTexture_H

#include "SceneManager.h"

#include <osg/ImageStream>
#include <osg/ImageSequence>
#include <osg/TextureRectangle>
#include <osg/Texture2D>
#include <osg/Timer>


/**
 * \brief A video texture node that allows dynamic textures to be applied
 *
 * This uses osg::ImageStream, so it can load several different movie formats,
 * including live feeds on the Mac via devices.live.
 */
//class VideoTexture : virtual public osg::TextureRectangle
class VideoTexture : virtual public osg::StateSet
{

public:

	VideoTexture(const char *initID);
	~VideoTexture();

	//void updateCallback();
	
	void debugPrint ();
	
	/**
	 * Creates a video from a path on disk. This can either be a single movie
	 * file (ie, a format that OSG knows how to read), or a folder name that
	 * contains a sequence of images to stitch into a video.
	 */
	void setVideoPath (const char* newPath);
	const char *getVideoPath() { return _path.c_str(); }

	
	void setFlip (int i);
	int getFlip() { return (int)_flip; }
	
	/**
	 * Enable (1) or disable (0) looping of the video
	 */
	void setLoop (int i);
	int getLoop() { return (int)_loop; }

	/**
	 * Seek to a particular part of the movie
	 */
	void setIndex (float f);
	float getIndex() { return _index; }

	/**
	 * Only for image sequences; tells OSG how fast to play the sequence
	 */
	void setFrameRate (float f);
	
	/**
	 * Play (1) or Pause (0) the video
	 */
	void setPlay (int i);
	int getPlay() { return (int)_play; }

	/**
	 * Seek to beginning of video (quivalent to setIndex(0);
	 */
	void rewind ();
	
	
	/**
	 * Returns whether there is a currently valid video
	 */
	bool isValid() { return (_imageStream.valid()); }
	//bool isValid() { return (_imageStream.valid() || _imageSequence.valid()); }

private:
	
	t_symbol *_id;
	
	std::string _path;
	bool _flip, _loop, _play;
	float _index;
	
	bool _useTextureRectangle;
	bool _isSequence;
	
	osg::ref_ptr<osg::ImageStream> _imageStream;
	//osg::ref_ptr<osg::ImageSequence> _imageSequence;
};

/*
class VideoTexture_callback : public osg::StateAttribute::StateAttribute::Callback
{

	public:
		virtual void operator()(osg::StateAttribute* attr, osg::NodeVisitor* nv)
		{
			osg::ref_ptr<VideoTexture> thisAttr = dynamic_cast<VideoTexture*> (attr->getUserData());

			if (thisAttr != NULL)
			{
				thisAttr->updateCallback();
			}
		}
};
*/

#endif
