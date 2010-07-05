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

//#include <osg/Texture>
#include <osg/TexMat>
#include <osg/TexEnv>
#include <osg/StateSet>
#include <osg/StateAttribute>


#include <iostream>



#include "videoSize.h"
#include "osgUtil.h"
#include "SharedVideoNode.h"
#include "SceneManager.h"
#include "MediaManager.h"


static const GLenum PIXEL_TYPE = GL_UNSIGNED_SHORT_5_6_5;

using namespace std;

// ===================================================================
// constructor:
SharedVideoNode::SharedVideoNode (SceneManager *sceneManager, char *initID) : ShapeNode(sceneManager, initID)
{
    this->setName(string(id->s_name) + ".SharedVideoNode");
    nodeType = "SharedVideoNode";

    this->setBillboard(ShapeNode::STAY_UP);

    // worker thread is in killed state to start:
    killed_ = true;

    textureID = "NULL";

    width=640;
    height=480;

    drawShape(); // will automatically call drawTexture()
}

// ===================================================================
// destructor
SharedVideoNode::~SharedVideoNode()
{

}


void SharedVideoNode::callbackUpdate()
{

    // do update here
    ReferencedNode::callbackUpdate();

    if (!killed_ && sceneManager->isGraphical() && textureImage.valid() && textureRect.valid())
    {
        boost::mutex::scoped_lock displayLock(displayMutex_);

        // update image from shared memory:
        textureImage->setImage(width,
                height,
                0,
                GL_RGB,
                GL_RGB,
                PIXEL_TYPE,
                sharedBuffer->pixelsAddress(),
                osg::Image::NO_DELETE,
                1);

        // set texture:
        textureRect->setImage(textureImage.get());

        // flip image from camera space:
        //textureImage->flipHorizontal();
        //textureImage->flipVertical();

        //textureImage->setOrigin(osg::Image::TOP_LEFT);


        textureUploadedCondition_.notify_one();

    }


}


/// This function is executed in the worker thread
void SharedVideoNode::consumeFrame()
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
    //shared_memory_object::remove(textureID.c_str());
    // TODO: shouldn't we also destroy our shm and region objects?
}

// ===================================================================
void SharedVideoNode::signalKilled ()
{
    boost::mutex::scoped_lock displayLock(displayMutex_);
    killed_ = true;
    textureUploadedCondition_.notify_one(); // in case we're waiting in consumeFrame
}

// ===================================================================
void SharedVideoNode::setHost (const char *newvalue)
{
    // need to redraw after setHost() is called:
    if (host != string(newvalue))
    {
        ReferencedNode::setHost(newvalue);
        drawShape();
    }
}

// ===================================================================
void SharedVideoNode::setTextureID (const char* newID)
{

    std::cout << "switching videoTexture for node " << this->id->s_name << " ... from " << textureID << " to " << newID << std::endl;

    // only do this if the id has changed:
    if (textureID == std::string(newID)) return;
    textureID = std::string(newID);


    bool ignoreOnThisHost = ( !sceneManager->isGraphical() || (host==getHostname()) );
    if (!ignoreOnThisHost)
    {
        if (!killed_)
        {
            // first kill any existing thread:
            this->signalKilled(); // let worker know that the mainloop has exitted
            worker.join(); // wait for worker to end
        }

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
            sharedBuffer = static_cast<SharedVideoBuffer*>(addr);

            width = sharedBuffer->getWidth();
            height = sharedBuffer->getHeight();

            // reset the killed_ conditional
            killed_ = false;

            // start our consumer thread, which is a member function of this class
            // and takes sharedBuffer as an argument
            worker = boost::thread(boost::bind<void>(boost::mem_fn(&SharedVideoNode::consumeFrame), boost::ref(*this)));
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
                std::cerr << "Tried to setTextureID for " << this->id->s_name << ", but shared buffer " << textureID << " doesn't exist yet\n";
                //boost::this_thread::sleep(boost::posix_time::milliseconds(30));
            }
            killed_ = true;
        }


    }


    BROADCAST(this, "ss", "setTextureID", getTextureID());



}

// ===================================================================
void SharedVideoNode::drawShape()
{
    // Then check if the new value is equivalent to this machine's hostname.
    // If so, prevent it from drawing anything (but only for client threads)
    if (not spinApp::Instance().getContext->isServer() && (host==getHostname()))
    {
        this->setShape(ShapeNode::NONE);
    } else {
        this->setShape(ShapeNode::PLANE);
    }

    // must explicitly call drawShape() in base class
    ShapeNode::drawShape();

}

// ===================================================================
void SharedVideoNode::drawTexture()
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

        // from ShapeNode:
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

        textureImage->setOrigin(osg::Image::TOP_LEFT);

        osg::StateSet *shapeStateSet = new osg::StateSet();
        //osg::StateSet *shapeStateSet = shapeGeode->getOrCreateStateSet();

        // setup texture
        textureRect = new osg::TextureRectangle(textureImage.get());
        textureRect->setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST);
        textureRect->setFilter(osg::Texture::MAG_FILTER, osg::Texture::NEAREST);
        textureRect->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
        textureRect->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
        shapeStateSet->setTextureAttributeAndModes(0, textureRect.get(), osg::StateAttribute::ON);

        // Switch on the post scaling of the TexMat matrix by the size of the
        // last applied texture rectangle.
        osg::ref_ptr<osg::TexMat> texmat(new osg::TexMat);
        texmat->setScaleByTextureRectangleSize(true);
        shapeStateSet->setTextureAttributeAndModes(0, texmat.get(), osg::StateAttribute::ON);

        // Mike thinks this TexEnv not necessary for a single texture. However,
        // when multi-texturing, it means that we REPLAVE underlying color with
        // our video texture color, instead of blending (ADD, MODULATE, etc.)
        osg::ref_ptr<osg::TexEnv> texenv(new osg::TexEnv);
        texenv->setMode(osg::TexEnv::REPLACE);
        shapeStateSet->setTextureAttributeAndModes(0, texenv.get(), osg::StateAttribute::ON);

        // turn off lighting
        shapeStateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

        shapeGeode->setStateSet( shapeStateSet );

    }

}

std::vector<lo_message> SharedVideoNode::getState ()
{
    // inherit state from base class
    std::vector<lo_message> ret = ShapeNode::getState();

    lo_message msg;

    msg = lo_message_new();
    lo_message_add(msg, "ss", "setTextureID", getTextureID());
    ret.push_back(msg);

    return ret;
}

