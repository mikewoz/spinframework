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

#include "Shader.h"

#ifdef WITH_SHARED_VIDEO 
#include <shmdata/osg-reader.h>
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
 * \brief Allows reception of video from an other process into an OSG texture.
 *
 * This is accomplished by the use of libshmdata. 
 * http://code.sat.qc.ca/redmine/projects/libshmdata
 *
 * Note however, that this node will be available regardless of the library's
 * presence. This is because we still want to know about it on ALL servers or
 * clients, regardless of platform or available libraries.
 *
 * ie, this node still has reduced funtionality on non-supported platforms.
 */


class SharedVideoTexture : public Shader
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
    
    
    //void updateCallback();
    //void consumeFrame();
    //void signalKilled();

    void start(); 
    void stop();

private:
    
    std::string textureID;
    
    osg::ref_ptr<osg::Texture2D> tex;
    osg::ref_ptr<osg::Image> img;
    
    int width, height;
        
    osg::Timer_t lastTick;
        
    bool killed_;
    
#ifdef WITH_SHARED_VIDEO         
    shmdata::OsgReader *reader_;
/*     boost::thread worker_; */
/*     boost::mutex displayMutex_; */
/*     boost::condition_variable textureUploadedCondition_; */
/*     SharedVideoBuffer *sharedBuffer; */
#endif 
	
};


} // end of namespace spin

#endif
