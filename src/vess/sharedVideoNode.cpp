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

//#include <osg/Texture>
#include <osg/TexMat>
#include <osg/TexEnv>
#include <osg/StateSet>
#include <osg/StateAttribute>


#include <iostream>




#include "asGlobals.h"
#include "osgUtil.h"
#include "sharedVideoNode.h"
#include "asSceneManager.h"
#include "asMediaManager.h"


static const GLenum PIXEL_TYPE = GL_UNSIGNED_SHORT_5_6_5;

using namespace std;


extern pthread_mutex_t pthreadLock;


// ===================================================================
// constructor:
sharedVideoNode::sharedVideoNode (asSceneManager *sceneManager, char *initID) : asShape(sceneManager, initID)
{
	this->setName(string(id->s_name) + ".sharedVideoNode");
	nodeType = "sharedVideoNode";
	

	this->setShape(asShape::SPHERE);
		
	textureID = "NULL";

}

// ===================================================================
// destructor
sharedVideoNode::~sharedVideoNode()
{

}


void sharedVideoNode::callbackUpdate()
{

    // do update here


    if (sceneManager->isGraphical() && textureImage.valid() && textureRect.valid())
    {
    	std::cout << "updating texture" << std::endl;
    	
        boost::mutex::scoped_lock displayLock(displayMutex_);

	    // update image from shared memory:
	    textureImage->setImage(SharedVideoBuffer::WIDTH, 
	    		SharedVideoBuffer::HEIGHT, 
	            0, 
	            GL_RGB, 
	            GL_RGB, 
	            PIXEL_TYPE, 
	            sharedBuffer->pixelsAddress(), 
	            osg::Image::NO_DELETE, 
	            1);
	
	    // set texture:
	    textureRect->setImage(textureImage.get());
	    
	    textureUploadedCondition_.notify_one();
	    
    }
}


/// This function is executed in the worker thread
void sharedVideoNode::consumeFrame()
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
	            sharedBuffer->removeSentinel();   // tell appsink to give us buffers
	    }


	    do
	    {
	        {
	            // Lock the mutex
	            scoped_lock<interprocess_mutex> lock(sharedBuffer->getMutex());

	            sharedBuffer->removeSentinel();   // tell appsink not to give us any more buffers

	            // wait for new buffer to be pushed if it's empty
	            sharedBuffer->waitOnProducer(lock);

	            if (sharedBuffer->hasSentinel())
	                end_loop = true;
	            else
	            {
	                // got a new buffer, wait until we upload it in gl thread before notifying producer
	                {
	                    boost::mutex::scoped_lock displayLock(displayMutex_);

	                    if (killed_)
	                    {
	                        sharedBuffer->pushSentinel();   // tell appsink not to give us any more buffers
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

	    std::cout << "\nWorker thread Going out.\n";
	    // erase shared memory
	    // shared_memory_object::remove("shared_memory");
}

// ===================================================================
void sharedVideoNode::signalKilled ()
{
    boost::mutex::scoped_lock displayLock(displayMutex_);
    killed_ = true;
    textureUploadedCondition_.notify_one(); // in case we're waiting in consumeFrame
}

// ===================================================================
void sharedVideoNode::setTextureID (const char* id)
{

	// only do this if the id has changed:
	if (textureID == std::string(id)) return;
	textureID = std::string(id);
	
	if (sceneManager->isGraphical())
	{
		
		// first kill any existing thread:
	    //this->signalKilled(); // let worker know that the mainloop has exitted
	    //worker.join(); // wait for worker to end
	
	    using namespace boost::interprocess;
	    try
	    {	
	        // open the already created shared memory object
	        shm = new shared_memory_object(open_only, textureID.c_str(), read_write);
	        
	        // map the whole shared memory in this process
	        region = new mapped_region(*shm, read_write);
	
	        // get the address of the region
	        void *addr = region->get_address();
	
	        // cast to pointer of type of our shared structure
	        this->sharedBuffer = static_cast<SharedVideoBuffer*>(addr);
	
	        // reset the killed_ conditional
	        killed_ = false;
	        
	        std::cout << "starting worker thread for " << textureID << std::endl;
	
	        // start our consumer thread, which is a member function of this class
	        // and takes sharedBuffer as an argument
	        worker = boost::thread(boost::bind<void>(boost::mem_fn(&sharedVideoNode::consumeFrame), boost::ref(this)));
	        
	        
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
	            std::cerr << "Shared buffer doesn't exist yet\n";
	            boost::this_thread::sleep(boost::posix_time::milliseconds(30)); 
	        }
	    }
    
	    drawTexture();
	}
    
    
    BROADCAST(this, "ss", "setTextureID", getTextureID());
    

    
}


// ===================================================================
void sharedVideoNode::drawTexture()
{
	/*
	if (textureID==0)
	{
		// remove current texture
		shapeGeode->setStateSet( new osg::StateSet() );
		return;
	}
	*/

	if (shapeGeode.valid())
	{

		// from asShape:
		/*
		osg::Texture2D* shapeTexture = new osg::Texture2D();
		shapeTexture->setBorderColor(osg::Vec4(1.0f,1.0f,1.0f,0.0f));
		shapeTexture->setImage(texturePointer.get());

		//osg::StateSet *shapeStateSet = new osg::StateSet();
		osg::StateSet *shapeStateSet = shapeGeode->getOrCreateStateSet();

		// Turn blending on:
		shapeStateSet->setMode( GL_BLEND, osg::StateAttribute::ON );
		shapeStateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

		// Disable depth testing so geometry is drawn regardless of depth values
		// of geometry already draw.
		//shapeStateSet->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);

		// Disable lighting:
		//shapeStateSet->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

		// Need to make sure this geometry is draw last. RenderBins are handled
		// in numerical order.
		shapeStateSet->setRenderBinDetails( renderBin, "RenderBin");

		// Set Texture:
		shapeStateSet->setTextureAttributeAndModes(0,shapeTexture,osg::StateAttribute::ON);

		//shapeGeode->setStateSet( shapeStateSet.get() );
		*/

		textureImage = new osg::Image;

	    // setup texture
	    textureRect = new osg::TextureRectangle(textureImage.get());
	    textureRect->setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST);
	    textureRect->setFilter(osg::Texture::MAG_FILTER, osg::Texture::NEAREST);
	    textureRect->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
	    textureRect->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);

	    osg::ref_ptr<osg::TexMat> texmat(new osg::TexMat);
	    texmat->setScaleByTextureRectangleSize(true);

	    osg::ref_ptr<osg::TexEnv> texenv(new osg::TexEnv);
	    texenv->setMode(osg::TexEnv::REPLACE);

	    // setup state for geometry
	    osg::StateSet *shapeStateSet = shapeGeode->getOrCreateStateSet();
	    shapeStateSet->setTextureAttributeAndModes(0, textureRect.get(), osg::StateAttribute::ON);
	    shapeStateSet->setTextureAttributeAndModes(0, texmat.get(), osg::StateAttribute::ON);
	    shapeStateSet->setTextureAttributeAndModes(0, texenv.get(), osg::StateAttribute::ON);

	    // turn off lighting 
	    shapeStateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	}

}

std::vector<lo_message> sharedVideoNode::getState ()
{
	// inherit state from base class
	std::vector<lo_message> ret = asShape::getState();

	lo_message msg;

	msg = lo_message_new();
	lo_message_add(msg, "ss", "setTextureID", getTextureID());
	ret.push_back(msg);

	return ret;
}

