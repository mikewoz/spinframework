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

#include "ReferencedStateSet.h"

class SceneManager;

namespace osg {
    class ImageStream;
}

/**
 * \brief A video texture node that allows dynamic textures to be applied
 *
 * This uses osg::ImageStream, so it can load several different movie formats,
 * including live feeds on the Mac via devices.live.
 */
class VideoTexture : public ReferencedStateSet
{

public:

	VideoTexture(SceneManager *sceneManager, const char *initID);
	~VideoTexture();

	//virtual void updateCallback();
	
	virtual void debug();
	
	/**
	 * Creates a video from a path on disk. This can either be a single movie
	 * file (ie, a format that OSG knows how to read), or a folder name that
	 * contains a sequence of images to stitch into a video.
	 */
	void setPath (const char* newPath);
	const char *getPath() const { return _path.c_str(); }

	/**
	 * Enable (1) or disable (0) looping of the video
	 */
	void setLoop (int i);
	int getLoop() const { return (int)_loop; }

	/**
	 * Normalized seek to a part of the video. ie, index in range [0,1]
	 */
	void setIndex (float f);
	float getIndex() const { return _index; }

	/**
	 * Only for image sequences; tells OSG how fast to play the sequence
	 */
	void setFrameRate (float f);
	float getFrameRate() const { return _framerate; }
	
	/**
	 * Play (1) or Pause (0) the video
	 */
	void setPlay (int i);
	int getPlay() const { return (int)_play; }

	
	/**
	 * Seek to beginning of video (quivalent to setIndex(0);
	 */
	void rewind();
	
	
	void flipHorizontal();
	void flipVertical();
	
	
	/**
	 * Returns whether there is a currently valid video
	 */
	bool isValid() { return (_imageStream.valid()); }
	//bool isValid() { return (_imageStream.valid() || _imageSequence.valid()); }

	// must reimplement
	virtual std::vector<lo_message> getState() const;

	
private:

	std::string _path;

	bool _flip, _loop, _play;
	
	float _framerate;
	float _index;
	
	bool _useTextureRectangle;
	bool _isSequence;
	
	osg::ref_ptr<osg::ImageStream> _imageStream;
	//osg::ref_ptr<osg::ImageSequence> _imageSequence;
};


#endif
