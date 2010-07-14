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

#include "config.h"

#include <iostream>
#include "SharedVideoTexture.h"
#include "SceneManager.h"
#include "spinApp.h"
#include "spinBaseContext.h"

static const GLenum PIXEL_TYPE = GL_UNSIGNED_SHORT_5_6_5;

using namespace std;



// ===================================================================
// constructor:
//SharedVideoTexture::SharedVideoTexture (const char *initID) : osg::TextureRectangle()
SharedVideoTexture::SharedVideoTexture  (SceneManager *s, const char *initID) : 
    ReferencedStateSet(s, initID),
    shm_(0),
    region_(0)
{
	classType = "SharedVideoTexture";
	
	
	// worker thread is in killed state to start:
	killed_ = true;
	
	width=640;
	height=480;

	// placeholder image:
	img = new osg::Image;
	//img->setOrigin(osg::Image::TOP_LEFT); 
	img->setOrigin(osg::Image::BOTTOM_LEFT); 
   
	// setup texture:
	tex = new osg::Texture2D; //(img.get());
	tex->setImage(img.get());
	tex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST);
    tex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::NEAREST);
    tex->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
    tex->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
	
	// It is important to disable resizing of this texture if
	// resolution is not a power of two. Othwerwise, there will
	// be a very slow resize every update. NOTE: this might cause
	// problems on harware that doesn't support NPOT textures.
	tex->setResizeNonPowerOfTwoHint(false);

		
	// add the texture to this (StateSet)
	this->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE );
	
	// turn off lighting 
	this->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	// keep a timer for reload attempts:
	lastTick = osg::Timer::instance()->tick();
	
	std::cout << "created SharedVideoTexture with id: " << this->id->s_name << std::endl;

	// set initial textureID:
	//setTextureID(this->id->s_name);
	
}

// ===================================================================
// destructor
SharedVideoTexture::~SharedVideoTexture()
{
	std::cout << "SharedVideoTexture destructor for id:" << this->id->s_name << std::endl;
#ifdef WITH_SHARED_VIDEO
	stop();
    delete region_;
    delete shm_;
#else
        std::cerr << "WARNING: SHARED_VIDEO not enabled\n";
#endif
}


// ===================================================================
void SharedVideoTexture::setTextureID (const char* newID)
{
	
	// only do this if the id has changed:
	if (textureID == std::string(newID)) return;
	
	textureID = std::string(newID);
	this->setName("SharedVideoTexture("+textureID+")");
	
	if (sceneManager->isGraphical())
	{
#ifdef WITH_SHARED_VIDEO
        if (region_)
        {
            delete region_;
            region_ = 0;
        }
        if (shm_)
        {
            delete shm_;
            shm_ = 0;
        }
		start();
#else
        std::cerr << "WARNING: SHARED_VIDEO not enabled\n";
#endif
	}
	
	else
	{
		BROADCAST(this, "ss", "setTextureID", getTextureID());
	}
}

// *****************************************************************************
std::vector<lo_message> SharedVideoTexture::getState ()
{
	// inherit state from base class
	std::vector<lo_message> ret = ReferencedStateSet::getState();
		
	lo_message msg;
	
	msg = lo_message_new();
	lo_message_add(msg, "ss", "setTextureID", getTextureID());
	ret.push_back(msg);

	return ret;
}


// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
// The rest of this stuff is only valid if we are using the shared_video library
// from Miville (configurable option in build)

#ifdef WITH_SHARED_VIDEO

void SharedVideoTexture::updateCallback()
{

    // do update here
    

    if (!killed_ && img.valid())
    {	
    	boost::mutex::scoped_lock displayLock(displayMutex_);
		
    	// update image from shared memory:
	    img->setImage(width, 
	    		height, 
	            0, 
	            GL_RGB, 
	            GL_RGB, 
	            PIXEL_TYPE, 
	            sharedBuffer->pixelsAddress(), 
	            osg::Image::NO_DELETE, 
	            1);
	    		
	    // set texture:
	    tex->setImage(img.get());

		// flip image from camera space:
		//img->flipHorizontal();
		//img->flipVertical();
	
	    //img->setOrigin(osg::Image::TOP_LEFT); 

	    
	    textureUploadedCondition_.notify_one();

    } else if (!textureID.empty()) {
		
		// if the shared memory has not been set, we will recheck
		// every 5 seconds or so
		
		osg::Timer_t tick = osg::Timer::instance()->tick();
		float dt = osg::Timer::instance()->delta_s(lastTick,tick);
		if (dt > 5)
		{
			start();
		
			// this is the last time we checked
			lastTick = osg::Timer::instance()->tick();
		}
	}
    
    
}
		

/// This function is executed in the worker thread
void SharedVideoTexture::consumeFrame()
{	
	using boost::interprocess::scoped_lock;
	using boost::interprocess::interprocess_mutex;
	using boost::interprocess::shared_memory_object;

	// get frames until the other process marks the end
	bool end_loop = false;

    // make sure there's no sentinel
    {
        // Lock the mutex
        scoped_lock<interprocess_mutex> lock(sharedBuffer->getMutex());
        sharedBuffer->startPushing();   // tell appsink to give us buffers
    }

    do
    {
        {
            // Lock the mutex
            scoped_lock<interprocess_mutex> lock(sharedBuffer->getMutex());

            // wait for new buffer to be pushed if it's empty
            sharedBuffer->waitOnProducer(lock);
			

            if (!sharedBuffer->isPushing())
                end_loop = true;
            else
            {
                // got a new buffer, wait until we upload it in gl thread before notifying producer
                {
                    boost::mutex::scoped_lock displayLock(displayMutex_);

                    if (killed_)
                    {
                        sharedBuffer->stopPushing();   // tell appsink not to give us any more buffers
                        end_loop = true;
                    }
                    else
                        textureUploadedCondition_.wait(displayLock);
                }

                // Notify the other process that the buffer status has changed
                sharedBuffer->notifyProducer();
            }
            // mutex is released (goes out of scope) here
        }
    }
    while (!end_loop);

	
    // erase shared memory
	// No! we never do this... only the other process is allowed to
	// destroy the memory
    //shared_memory_object::remove(textureID.c_str());
    // shouldn't we also destroy our shm and region objects?
}

// ===================================================================
void SharedVideoTexture::signalKilled ()
{
    boost::mutex::scoped_lock displayLock(displayMutex_);
    killed_ = true;
    textureUploadedCondition_.notify_one(); // in case we're waiting in consumeFrame
}

// ===================================================================
void SharedVideoTexture::start()
{
	// first kill any existing thread:
	stop();
	
	using namespace boost::interprocess;
	try
	{
		// open the already created shared memory object
        assert(shm_ == 0);
		shm_ = new shared_memory_object(open_only, textureID.c_str(), read_write);

		// map the whole shared memory in this process
        assert(region_ == 0);
		region_ = new mapped_region(*shm_, read_write);
		
		// get the address of the region
		void *addr = region_->get_address();
		
		
		// cast to pointer of type of our shared structure
		sharedBuffer = static_cast<SharedVideoBuffer*>(addr);
			
		width = sharedBuffer->getWidth();
		height = sharedBuffer->getHeight();
		
		// reset the killed_ conditional
		killed_ = false;
		
		std::cout << "SharedVideoTexture '" << textureID << "' starting thread" << std::endl;
		
		// start our consumer thread, which is a member function of this class
		// and takes sharedBuffer as an argument
		worker = boost::thread(boost::bind<void>(boost::mem_fn(&SharedVideoTexture::consumeFrame), boost::ref(*this)));
	}
	catch(interprocess_exception &ex)
	{
		static const char *MISSING_ERROR = "No such file or directory";
		if (strncmp(ex.what(), MISSING_ERROR, strlen(MISSING_ERROR)) != 0)
		{
			shared_memory_object::remove(textureID.c_str());
			std::cout << "Unexpected exception: " << ex.what() << std::endl;
		}
		else
		{
			std::cerr << "Tried to loadSharedMemory, but shared buffer " << textureID << " doesn't exist yet\n";
			//boost::this_thread::sleep(boost::posix_time::milliseconds(30)); 
		}
		killed_ = true;
	}

}

void SharedVideoTexture::stop()
{
	if (!killed_)
	{
		this->signalKilled();
		worker.join(); // wait here until thread exits
		std::cout << "SharedVideoTexture '" << textureID << "' stopped thread" << std::endl;
	}
}

#endif
