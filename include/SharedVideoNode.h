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

#ifndef __SharedVideoNode_H
#define __SharedVideoNode_H

#include <osg/TextureRectangle>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>
#include <boost/bind.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include "ShapeNode.h"
#include "sharedVideoBuffer.h"



/**
 * \brief This node shared a GL texture from memory with another process
 *
 * This is accomplished by the use of boost/interprocess/shared_memory_object
 */
class SharedVideoNode : public ShapeNode
{

public:

	SharedVideoNode(SceneManager *sceneManager, char *initID);
	virtual ~SharedVideoNode();

	virtual void callbackUpdate();
	
	// from tristan:
	void consumeFrame();
	void signalKilled();

	virtual void setHost(const char *newvalue);
	
	void setTextureID(const char *id);
	const char* getTextureID() { return textureID.c_str(); }
	
	/**
	 * For each subclass of ReferencedNode, we override the getState() method to
	 * fill the vector with the correct set of methods for this particular node
	 */
	virtual std::vector<lo_message> getState() const;


	
private:
	
	std::string textureID;
	osg::ref_ptr<osg::TextureRectangle> textureRect;

	osg::ref_ptr<osg::Image> textureImage;
	
	int width, height;
	
	// from tristan:
	boost::thread worker;
	boost::mutex displayMutex_;
	boost::condition_variable textureUploadedCondition_;
	SharedVideoBuffer *sharedBuffer;
	
	boost::interprocess::shared_memory_object *shm;
	boost::interprocess::mapped_region *region;    
    
	bool killed_;


	// must override draw methods:
	virtual void drawShape();
	virtual void drawTexture();

};


#endif
