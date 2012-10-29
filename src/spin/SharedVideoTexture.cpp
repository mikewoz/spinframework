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

#include <osg/Texture2D>
#include <osg/Image>

#include <iostream>
#include "sharedvideotexture.h"
#include "scenemanager.h"
#include "spinapp.h"
#include "spinbasecontext.h"

using namespace std;

namespace spin
{


  // ===================================================================
  // constructor:
  //SharedVideoTexture::SharedVideoTexture (const char *initID) : osg::TextureRectangle()
  SharedVideoTexture::SharedVideoTexture  (SceneManager *s, const char *initID) : 
    Shader(s, initID), killed_(true)
  {
    classType_ = "SharedVideoTexture";

    // placeholder image:
    img = new osg::Image;
    img->setOrigin(osg::Image::TOP_LEFT); 
    //img->setOrigin(osg::Image::BOTTOM_LEFT); 

    // setup texture:
#ifdef WITH_SHARED_VIDEO
    reader_.setDebug (true);
    tex = reader_.getTexture ();
#else
    tex = new osg::Texture2D; //(img.get());
#endif
    
    tex->setImage(img.get());

    tex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR);
    tex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
	
    //tex->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
    //tex->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
    if (textureRepeatS_)
      tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
    else
      tex->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP);
    if (textureRepeatT_)
      tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
    else
      tex->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP);

    osg::TexEnv* texenv = new osg::TexEnv();
    texenv->setMode(textureBlend_);
    this->setTextureAttribute(0, texenv, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE );
	
    // It is important to disable resizing of this texture if
    // resolution is not a power of two. Othwerwise, there will
    // be a very slow resize every update. NOTE: this might cause
    // problems on harware that doesn't support NPOT textures.
    tex->setResizeNonPowerOfTwoHint(false);

    // add the texture to this (StateSet)
    this->setTextureAttributeAndModes(0, tex, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE );

    // set lighting:
    if (lightingEnabled_) this->setMode( GL_LIGHTING, osg::StateAttribute::ON );
    else this->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

    // set renderbin:
    this->setRenderBinDetails( renderBin_, "RenderBin");
	
    // enable blending:
    this->setMode(GL_BLEND, osg::StateAttribute::ON);
    this->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    // disable depth test (FIXME)
    //this->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);

    // keep a timer for reload attempts:
    lastTick = 0;

    std::cout << "created shmdataVideoTexture with id: " << this->getID() << std::endl;

  }

  // ===================================================================
  // destructor
  SharedVideoTexture::~SharedVideoTexture()
  {
  }
  
  // ===================================================================
  // update callback
  void SharedVideoTexture::updateCallback()
  {
#ifdef WITH_SHARED_VIDEO
    reader_.updateImage();
#endif

    //((spin::Shader*)this)->updateCallback();
  }

  // ===================================================================
  void SharedVideoTexture::setTextureID (const char* newID)
  {

    // only do this if the id has changed:
    if (textureID == std::string(newID)) return;

    textureID = std::string(newID);
    this->setName("shmdataVideoTexture("+textureID+")");

    if (!sceneManager_->isGraphical())
      {
        BROADCAST(this, "ss", "setTextureID", getTextureID());
      }

#ifdef WITH_SHARED_VIDEO
   if (sceneManager_->isGraphical())
      {
	//start the shmdata
	reader_.setPath (textureID.c_str());
      }
#endif  


  }

  // *****************************************************************************
  std::vector<lo_message> SharedVideoTexture::getState () const
  {

    // inherit state from base class
    std::vector<lo_message> ret = Shader::getState();

    lo_message msg;

    msg = lo_message_new();
    lo_message_add(msg, "ss", "setTextureID", getTextureID());
    ret.push_back(msg);

    return ret;
  }

  void SharedVideoTexture::debug()
  {

    Shader::debug();
    std::cout << "   ---------" << std::endl;
    std::cout << "   Type: shmdataVideoTexture" << std::endl;
    std::cout << "   Texture ID: " << getTextureID() << std::endl;
    std::cout << "   Path: " << getPath() << std::endl;
    std::cout << "   Render bin: " << getRenderBin() << std::endl;
#ifdef WITH_SHARED_VIDEO
    if (sceneManager_->isGraphical())
      std::cout << "   width/height: " << reader_.getWidth() << "x" << reader_.getHeight() << std::endl;
#endif  
    std::cout << "   Killed: " << killed_ << std::endl;
    std::cout << "   Texture ID: " << textureID << std::endl;
  }

  // *****************************************************************************
  // *****************************************************************************
  // *****************************************************************************
  // The rest of this stuff is only valid if we are using the shmdata library

#ifdef WITH_SHARED_VIDEO

  void SharedVideoTexture::play()
  {

    if (!sceneManager_->isGraphical())
      {
	BROADCAST(this, "s", "play");
      }
    else
      {
	std::cout << "SharedVideoTexture '" << textureID << "' playing" << std::endl;
	reader_.play();
      }

  }

  void SharedVideoTexture::pause()
  {

    if (!sceneManager_->isGraphical())
      {
	BROADCAST(this, "s", "pause");
      }
    else
      {
	std::cout << "SharedVideoTexture '" << textureID << "' paused" << std::endl;
	reader_.pause ();
      }
  }

#else

  void SharedVideoTexture::play() 
  { 
    cout << "shmdata not enabled during compilation, not starting" << std::endl; 
  }
  
  void SharedVideoTexture::pause() 
  { 
    cout << "shmdata not enabled during compilation, not starting" << std::endl; 
  }
    
#endif

} // end of namespace spin

