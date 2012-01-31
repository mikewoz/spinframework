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

#ifndef __SharedVideoTexture_H
#define __SharedVideoTexture_H

#include "config.h"
#include <osg/Timer>

#include "ReferencedStateSet.h"

#ifdef WITH_SHARED_VIDEO        
#include <shared-video-0.6/sharedVideoBuffer.h>
//#include <sharedVideoBuffer.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>
#include <boost/bind.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#endif

namespace osg {
    class Texture2D;
    class Image;
}

// forward declaration
class SharedVideoBuffer;

namespace spin
{

/**
 * \brief Allows sharing of a dynamc (video) GL texture from another process
 *
 * This is accomplished by the use of Miville's shared_video library, which in
 * turn uses boost/interprocess/shared_memory_object.
 *
 * Miville must be poperly installed for this to work, which really only works
 * in Linux. See the following URL for more info:
 *
 * https://svn.sat.qc.ca/trac/miville
 *
 * Note however, that this node will be available regardless of the library's
 * presence. This is because we still want to know about it on ALL servers or
 * clients, regardless of platform or available libraries.
 *
 * ie, this node still has reduced funtionality on non-supported platforms.
 */


class SharedVideoTexture : public ReferencedStateSet
{

public:

    SharedVideoTexture(SceneManager *sceneManager, const char *initID);
    ~SharedVideoTexture();

    void setTextureID(const char *id);
    const char* getTextureID() const { return textureID.c_str(); }

    // hack to get SceneManager::createStateSet to work:
    const char* getPath() const { return textureID.c_str(); }
        
    std::vector<lo_message> getState () const;
    void debug();
    
#ifdef WITH_SHARED_VIDEO
    void updateCallback();
    
    void consumeFrame();
    void signalKilled();

    void start();
    void stop();
#endif
        
    /**
     * Set the render bin for this texture. The higher the number, the later it
     * gets processed (ie, it appears on top). Default renderBin = 11
     */
    void setRenderBin (int i);
    int getRenderBin() const { return _renderBin; }
    void setEnableRenderBin (int i);
    int getEnableRenderBin () const { return enableRenderBin_ ? 1 : 0; }

private:
    
    std::string textureID;
    
    osg::ref_ptr<osg::Texture2D> tex;
    osg::ref_ptr<osg::Image> img;
    
    int width, height;
        
    osg::Timer_t lastTick;
        
    bool killed_;
    

#ifdef WITH_SHARED_VIDEO        
    boost::thread worker_;
    boost::mutex displayMutex_;
    boost::condition_variable textureUploadedCondition_;
    SharedVideoBuffer *sharedBuffer;
#endif
        
    int  _renderBin;
    bool enableRenderBin_;
};

/*
class SharedVideoTexture_callback : public osg::StateAttribute::StateAttribute::Callback
{

    public:
        virtual void operator()(osg::StateAttribute* attr, osg::NodeVisitor* nv)
        {
            osg::ref_ptr<SharedVideoTexture> thisAttr = dynamic_cast<SharedVideoTexture*> (attr->getUserData());

            if (thisAttr != NULL)
            {
                thisAttr->updateCallback();
            }
        }
};
*/

} // end of namespace spin

#endif
