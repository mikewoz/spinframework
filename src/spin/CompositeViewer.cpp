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

#include <string>
#include <iostream>
#include <osgDB/FileNameUtils>

#include <osg/TextureCubeMap>

#include <osgGA/NodeTrackerManipulator>
#include <osgGA/TrackballManipulator>
#include <osgGA/StateSetManipulator>

#include <osgDB/ReadFile>

#include "config.h"
#include "viewermanipulator.h"
#include "spinutil.h"
#include "spinapp.h"
#include "spinclientcontext.h"
#include "osgutil.h"
#include "groupnode.h"
#include "scenemanager.h"
#include "shapenode.h"
#include "compositeviewer.h"

#ifdef HAVE_SPNAV_H
#include "spnav.h"
#endif


extern pthread_mutex_t sceneMutex;


namespace spin
{


// constructor
CompositeViewer::CompositeViewer(osg::ArgumentParser& args) : osgViewer::CompositeViewer(args)
//CompositeViewer::CompositeViewer(osg::ArgumentParser& args) : osgViewer::Viewer(args)
{
    mbInitialized = false;
    mIsDome = false;
    mOldTime = 0.0f;
    lastNavTick_ = osg::Timer::instance()->tick();
    velocityScalars_ = osg::Vec3(1,1,1);
    spinScalars_ = osg::Vec3(1,1,1);
    mTexCubeMap = NULL;
}

CompositeViewer::~CompositeViewer()
{
    bool lIsDof = (mDofPPUs.size() == getNumViews());
    bool lIsSSAO = (mSsaoPPUs.size() == getNumViews());
    bool lIsMBlur = (mMBlurPPUs.size() == getNumViews());
    bool lIsOutline = (mOutlinePPUs.size() == getNumViews());
    bool lIsMask = (mMaskPPUs.size() == getNumViews());
    bool lIsShader = (mShaderPPUs.size() == getNumViews());

    if(mbInitialized)
    {
        for(unsigned int i=0; i<getNumViews(); ++i)
        {
            delete mProcessors[i];

            if(lIsDof)
                delete mDofPPUs[i];
            if(lIsSSAO)
                delete mSsaoPPUs[i];
            if(lIsMBlur)
                delete mMBlurPPUs[i];
            if(lIsOutline)
                delete mOutlinePPUs[i];
            if(lIsMask)
                delete mMaskPPUs[i];
            if(lIsShader)
                delete mShaderPPUs[i];
        }
    }
}

osg::Texture* CompositeViewer::createRenderTexture(int tex_width, int tex_height, bool depth, bool cubemap)
{
    osg::Texture2D* texture = new osg::Texture2D;
    if(!cubemap)
    {
        // create simple 2D texture
        texture->setTextureSize(tex_width, tex_height);
        texture->setResizeNonPowerOfTwoHint(false);
        texture->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
        texture->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
        texture->setWrap(osg::Texture2D::WRAP_S,osg::Texture2D::CLAMP_TO_EDGE);
        texture->setWrap(osg::Texture2D::WRAP_T,osg::Texture2D::CLAMP_TO_EDGE);
        texture->setBorderColor(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
    }
    else
    {
        // create a cubemap
        texture->setTextureSize(tex_width, tex_height);
        texture->setResizeNonPowerOfTwoHint(false);
        texture->setFilter(osg::Texture::MIN_FILTER,osg::Texture::LINEAR_MIPMAP_LINEAR);
        texture->setFilter(osg::Texture::MAG_FILTER,osg::Texture::LINEAR_MIPMAP_LINEAR);
        texture->setWrap(osg::Texture::WRAP_S,osg::Texture::CLAMP_TO_EDGE);
        texture->setWrap(osg::Texture::WRAP_T,osg::Texture::CLAMP_TO_EDGE);
        texture->setWrap(osg::Texture::WRAP_R,osg::Texture::CLAMP_TO_EDGE);
        texture->setBorderColor(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
    }

    // setup float format
    if (!depth)
    {
        texture->setInternalFormat(GL_RGBA16F_ARB);
        texture->setSourceFormat(GL_RGBA);
        texture->setSourceType(GL_FLOAT);
        /*
        texture->setInternalFormat(GL_RGBA32I_EXT);
        texture->setSourceFormat(GL_RGBA_INTEGER_EXT);
        texture->setSourceType(GL_BYTE);
        */
    }
    else{
        texture->setInternalFormat(GL_DEPTH_COMPONENT);
    }

    return texture;
}

void CompositeViewer::setupCamera()
{
    if(mIsDome == false)
    {
        for(unsigned int i=0; i<getNumViews(); ++i)
        {
            osgViewer::View* lView;
            osg::Camera* lCamera;
            
            lView = getView(i);
            lCamera = lView->getCamera();

            int lWidth = lCamera->getViewport()->width();
            int lHeight = lCamera->getViewport()->height();

            std::cout << std::endl;
            std::cout << "---------------------------------------" << std::endl;
            std::cout << "setting up camera " << i << " with wxh=" << lWidth<<"x"<<lHeight << std::endl;


            // create texture to render to
            osg::ref_ptr<osg::Texture> colorTexture1 = createRenderTexture(lWidth, lHeight, false, false);
            osg::ref_ptr<osg::Texture> colorTexture2 = createRenderTexture(lWidth, lHeight, false, false);
            osg::ref_ptr<osg::Texture> colorTexture3 = createRenderTexture(lWidth, lHeight, false, false);
            osg::ref_ptr<osg::Texture> colorTexture4 = createRenderTexture(lWidth, lHeight, false, false);
            osg::ref_ptr<osg::Texture> depthTexture = createRenderTexture(lWidth, lHeight, true, false);

            // set up the background color and clear mask.
            lCamera->setClearColor(osg::Vec4(0.0f,0.0f,0.0f,0.0f));
            lCamera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            // set viewport
            //lCamera->setViewport(lCamera->getViewport()); // Useful ??
            lCamera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
            lCamera->setCullingMode(osg::CullSettings::NO_CULLING);
            //camera->setProjectionMatrixAsPerspective(20.0, vp->width()/vp->height(), 0.1, 100.0);

            // tell the camera to use OpenGL frame buffer object where supported.
            lCamera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);

            // attach the texture and use it as the color buffer.
            //lCamera->detach(osg::Camera::COLOR_BUFFER);
            lCamera->attach(osg::Camera::COLOR_BUFFER0, colorTexture1);
            lCamera->attach(osg::Camera::COLOR_BUFFER1, colorTexture2);
            lCamera->attach(osg::Camera::COLOR_BUFFER2, colorTexture3);
            lCamera->attach(osg::Camera::COLOR_BUFFER3, colorTexture4);
            lCamera->attach(osg::Camera::DEPTH_BUFFER, depthTexture);
        }
    }
    else // dome view
    {
        // Now we iterate through the cameras
        // One of them is not composing the dome view, we detect it and don't do anything to it
        for(int i=0; i<getView(0)->getNumSlaves(); i++)
        {
            osg::Camera* lCamera = getView(0)->getSlave(i)._camera.get();

            if(lCamera->getRenderTargetImplementation() != osg::Camera::FRAME_BUFFER_OBJECT)
                continue;

            int lWidth = lCamera->getViewport()->width();
            int lHeight = lCamera->getViewport()->height();

            // We create 5 cubemap texture for FBO rendering, to replace the actual single one
            osg::ref_ptr<osg::Texture> colorTexture1 = createRenderTexture(lWidth, lHeight, false, false); 
            osg::ref_ptr<osg::Texture> colorTexture2 = createRenderTexture(lWidth, lHeight, false, false); 
            osg::ref_ptr<osg::Texture> colorTexture3 = createRenderTexture(lWidth, lHeight, false, false); 
            osg::ref_ptr<osg::Texture> colorTexture4 = createRenderTexture(lWidth, lHeight, false, false); 
            osg::ref_ptr<osg::Texture> depthTexture = createRenderTexture(lWidth, lHeight, true, false);

            std::cout << std::endl;
            std::cout << "---------------------------------------" << std::endl;
            std::cout << "setting up camera " << i << " with wxh=" << lWidth<<"x"<<lHeight << std::endl;

            lCamera->setClearColor(osg::Vec4(0.f, 0.f, 0.f, 0.f));
            lCamera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            lCamera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
            lCamera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);

            // We need to get the current attachment parameters
            //osg::Camera::BufferAttachmentMap attachmentMap = lCamera->getBufferAttachmentMap();
            //osg::Camera::Attachment* attachment = &(attachmentMap[osg::Camera::COLOR_BUFFER]);
            //bool mipmapGeneration = attachment->_mipMapGeneration;
            //unsigned int multisampleSamples = attachment->_multisampleSamples;
            //unsigned int multisampleColorSamples = attachment->_multisampleColorSamples;

            // Attach the textures to the color and depth buffers
            //lCamera->detach(osg::Camera::COLOR_BUFFER);
            //lCamera->attach(osg::Camera::COLOR_BUFFER0, colorTexture1, 0, 0, mipmapGeneration, multisampleSamples, multisampleColorSamples);
            //lCamera->attach(osg::Camera::COLOR_BUFFER1, colorTexture2, 0, 0, mipmapGeneration, multisampleSamples, multisampleColorSamples);
            //lCamera->attach(osg::Camera::COLOR_BUFFER2, colorTexture3, 0, 0, mipmapGeneration, multisampleSamples, multisampleColorSamples);
            //lCamera->attach(osg::Camera::COLOR_BUFFER3, colorTexture4, 0, 0, mipmapGeneration, multisampleSamples, multisampleColorSamples);

            lCamera->detach(osg::Camera::COLOR_BUFFER);
            lCamera->attach(osg::Camera::COLOR_BUFFER0, colorTexture1);
            lCamera->attach(osg::Camera::COLOR_BUFFER1, colorTexture2);
            lCamera->attach(osg::Camera::COLOR_BUFFER2, colorTexture3);
            lCamera->attach(osg::Camera::COLOR_BUFFER3, colorTexture4);

            lCamera->attach(osg::Camera::DEPTH_BUFFER, depthTexture);
        }
    }
}

//! Just setup some stuff
void CompositeViewer::viewerInit()
{
    // propagate the method
    osgViewer::CompositeViewer::viewerInit();
    //osgViewer::Viewer::viewerInit();

    // Check wether this is a dome view or not
    if(getView(0)->getNumSlaves() > 0)
        if(getView(0)->getSlave(0)._camera.get()->getBufferAttachmentMap()[osg::Camera::COLOR_BUFFER]._texture->getNumImages() == 6)
        {
            mIsDome = true;
            // We need to have access to the current cubemap attached to the slave cameras
            mTexCubeMap = (osg::TextureCubeMap*)(getView(0)->getSlave(0)._camera.get()->getBufferAttachmentMap()[osg::Camera::COLOR_BUFFER]._texture.get());
        }

    // setup data
    //setupCamera(getCamera()->getViewport());
    setupCamera();

    // add ppu processor into the scene graph
    osg::Node* lData = getView(0)->getSceneData();
    for(unsigned int i=0; i<getNumViews(); ++i)
    {
        osg::Group* group = new osg::Group();
        //group->addChild(getSceneData());
        group->addChild(lData);

        //setSceneData(group);
        this->getView(i)->setSceneData(group);
    }
}


#define MASK_ALL   0x00ff
#define MASK_SCENE 0x0010
#define MASK_MASK  0x0020

// WORKING  0x2 0xf
// NOT 10, 20

//#define MASK_SCENE MASK_ALL
//#define MASK_MASK  MASK_ALL


//! Setup osgppu for rendering
void CompositeViewer::initializePPU(unsigned int pEffect)
{
    if(pEffect == PPU_NONE)
        return;

    // if already initialized then just do nothing
    if (mbInitialized == false)
        mbInitialized = true;
    else
        return;

    // For each view, we create a processor
    // If this is a dome view, we iterate through slaves camera and not views
    // Moreoever, we need the output cubemap texture
    int lNbr = 0;
    if(mIsDome)
        lNbr = getView(0)->getNumSlaves();
    else
        lNbr = getNumViews();

    for(unsigned int i=0; i<lNbr; ++i)
    {
        osgPPU::Processor* lProcessor;
        lProcessor = new osgPPU::Processor();

        osgViewer::View* lView;
        osg::Camera* lCamera;

        osg::Texture* lOutTex = NULL;

        if(!mIsDome)
        {
            lView = getView(i);

            lCamera = lView->getCamera();
        }
        else
        {
            lView = getView(0);

            lCamera = lView->getSlave(i)._camera.get();
            // If not a camera from the dome
            if(lCamera->getRenderTargetImplementation() != osg::Camera::FRAME_BUFFER_OBJECT)
                continue;
        }
        
        osg::Group* lGroup = dynamic_cast<osg::Group*>(lView->getSceneData());
        lGroup->addChild( lProcessor );

        // initialize the post process
        lProcessor->setCamera(lCamera);
        
        lProcessor->setName("Processor");
        lProcessor->dirtyUnitSubgraph();

        mProcessors.push_back(lProcessor);

        osgPPU::Unit* lastUnit = NULL;

        /**********************************************/
        // SSAO effect must be applied before any other effect, especially those
        // which destroy information (such as DoF)
        if((pEffect & PPU_SSAO) != 0)
        {
            SSAORendering* lSsao = new SSAORendering();

            // Gets the projection matrix
            osg::Matrixf lProjectionMatrix = lCamera->getProjectionMatrix();
            lSsao->createSSAOPipeline(lProcessor, lastUnit, lProjectionMatrix);

            mSsaoPPUs.push_back(lSsao);
        }

        // Outline should be done before any blurring effect
        if((pEffect & PPU_OUTLINE) != 0)
        {
            OutlineRendering* lOutline = new OutlineRendering();

            double left,right,bottom,top,near,far;
            lCamera->getProjectionMatrixAsFrustum(left,right,bottom,top,near,far);
            lOutline->createOutlinePipeline(lProcessor, lastUnit, near, far);

            mOutlinePPUs.push_back(lOutline);
        }

        // DoF effect
        if((pEffect & PPU_DOF) != 0)
        {
            DoFRendering* lDoF = new DoFRendering();

            double left,right,bottom,top,near,far;
            lCamera->getProjectionMatrixAsFrustum(left,right,bottom,top,near,far);
            lDoF->createDoFPipeline(lProcessor, lastUnit, near, far);
            lDoF->setFocalLength(0.0);
            lDoF->setFocalRange(50.0);

            mDofPPUs.push_back(lDoF);
        }

        // Motion blur effect
        if((pEffect & PPU_MOTIONBLUR) != 0)
        {
            MotionBlurRendering* lMBlur = new MotionBlurRendering();
            lMBlur->createMotionBlurPipeline(lProcessor, lastUnit);

            mMBlurPPUs.push_back(lMBlur);
        }

        // Mask effect
        if((pEffect & PPU_MASK) != 0)
        {
            int xsize = lCamera->getViewport()->width();
            int ysize = lCamera->getViewport()->height();
            osg::Texture* texture2D = CompositeViewer::createRenderTexture(xsize, ysize, false, false);
            osg::Texture* textureDepth = CompositeViewer::createRenderTexture(xsize, ysize, true, false);
            // We need the depth buffer to not be smoothed, so we choose a nearest filter
            textureDepth->setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST);
            textureDepth->setFilter(osg::Texture::MAG_FILTER, osg::Texture::NEAREST);

            osg::Camera *slaveCam = new osg::Camera;

            slaveCam->addChild( spinApp::Instance().sceneManager_->rootNode.get() );
            lCamera->addChild( slaveCam ); // The slavecam needs to be child of the camera, for PPU to work

            //slaveCam->setInheritanceMask( osg::CullSettings::ALL_VARIABLES & ~osg::CullSettings::CULL_MASK );
            slaveCam->setCullMask( MASK_MASK );
            slaveCam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            slaveCam->setClearColor(osg::Vec4(0,0,0,0));
            slaveCam->setViewport( lCamera->getViewport() );
            slaveCam->setReferenceFrame(osg::Transform::RELATIVE_RF);
            slaveCam->setRenderOrder(osg::Camera::PRE_RENDER);
            slaveCam->setRenderTargetImplementation( osg::Camera::FRAME_BUFFER_OBJECT);
            slaveCam->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
            slaveCam->attach(osg::Camera::COLOR_BUFFER0, texture2D);
            slaveCam->attach(osg::Camera::DEPTH_BUFFER, textureDepth);

            lCamera->getGraphicsContext()->getState()->setCheckForGLErrors(osg::State::ONCE_PER_ATTRIBUTE);
            // lCamera->setInheritanceMask( osg::CullSettings::ALL_VARIABLES & ~osg::CullSettings::CULL_MASK );
            lCamera->setCullMask( MASK_SCENE );

            printf("cam getCullMask: %u 0x%x\n", lCamera->getCullMask(), lCamera->getCullMask() );
            printf("slaveCam getCullMask: %u 0x%x\n", slaveCam->getCullMask(), slaveCam->getCullMask() );

            MaskRendering* lMask = new MaskRendering();

            lMask->createMaskPipeline(lProcessor, lastUnit, slaveCam);

            mMaskPPUs.push_back(lMask);
        }

        // User specified shader
        if((pEffect & PPU_SHADER) != 0)
        {
            ShaderRendering* lShader = new ShaderRendering();
            lShader->createShaderPipeline(lProcessor, lastUnit);

            mShaderPPUs.push_back(lShader);
        }

        // add a text ppu after the pipeline is setted up
        /*if (0)
        {
            osgPPU::UnitText* fpstext = new osgPPU::UnitText();
            fpstext->setName("FPSTextPPU");
            fpstext->setSize(44);
            fpstext->setText("Example DoF-pipeline from a .ppu file");
            fpstext->setPosition(0.01, 0.95);
            lastUnit->addChild(fpstext);
        }*/

        // As a last step we setup a ppu which do render the content of the result
        // on the screenbuffer. This ppu MUST be as one of the last, otherwise you
        // will not be able to get results from the ppu pipeline
        osgPPU::Unit* ppuout;

        if(!mIsDome)
        {
            ppuout = new osgPPU::UnitOut();
            ppuout->setName("PipelineResult");
            ppuout->setInputTextureIndexForViewportReference(-1); // need this here to get viewport from camera
        }
        else
        {
            ppuout = new osgPPU::UnitInOut();
            ppuout->setName("PipelineResult");
            ppuout->setInputTextureIndexForViewportReference(-1); // need this here to get viewport from camera
            ((osgPPU::UnitInOut*)ppuout)->setOutputTextureType(osgPPU::UnitInOut::TEXTURE_CUBEMAP);

            //unsigned int lFace = lCamera->getBufferAttachmentMap()[osg::Camera::COLOR_BUFFER]._face;
            unsigned int lFace;
            switch(i)
            {
            case 0: lFace = osg::TextureCubeMap::POSITIVE_Y; break;
            case 1: lFace = osg::TextureCubeMap::POSITIVE_Z; break;
            case 2: lFace = osg::TextureCubeMap::NEGATIVE_X; break;
            case 3: lFace = osg::TextureCubeMap::POSITIVE_X; break;
            case 4: lFace = osg::TextureCubeMap::NEGATIVE_Z; break;
            case 5: lFace = osg::TextureCubeMap::NEGATIVE_Y; break;
            default: lFace = osg::TextureCubeMap::POSITIVE_Y; break;
            }

            ((osgPPU::UnitInOut*)ppuout)->setOutputFace(lFace);
            ((osgPPU::UnitInOut*)ppuout)->setOutputTexture(mTexCubeMap, 0);
        }
        lastUnit->addChild(ppuout);

        // write pipeline to a file
        //osgDB::writeObjectFile(*mProcessor, "dof.ppu");
    }
}

void CompositeViewer::updateSpaceNavigator()
{
#ifdef HAVE_SPNAV_H

    spin::spinApp &spin = spin::spinApp::Instance();

    float dt = (float)(osg::Timer::instance()->delta_s(lastNavTick_, osg::Timer::instance()->tick()) > 1.0);

    // TODO: frequency limiter:
    // (wait until at least 0.05 sec has transpired - ie, 20Hz)
    /*
    if (dt < 0.05)
    {
        spnav_remove_events(SPNAV_EVENT_MOTION);
        return;
    }
    */

    // poll the space navigator:
    int speventCount = 0;
    osg::Vec3 spVel, spSpin;
    float x=0;
    float y=0;
    float z=0;
    float rx=0;
    float ry=0;
    float rz=0;


    // if the last significant event was more than a second ago,
    // assume the user has let go of the puck and reset speedScale
    if ((dt > 1.0) && moving_)
    {
        //std::cout << "reset spacenavigator" << std::endl;
        spin.NodeMessage(spnavNodeID_.c_str(), "sfff", "setVelocity", 0.0, 0.0, 0.0, SPIN_ARGS_END);
        spin.NodeMessage(spnavNodeID_.c_str(), "sfff", "setSpin", 0.0, 0.0, 0.0, SPIN_ARGS_END);
        speedScaleValue_ = 1.0;
        moving_ = false;
    }

    spnav_event spnavevent;
    while (spnav_poll_event(&spnavevent))
    {
        if (spnavevent.type == SPNAV_EVENT_MOTION)
        {
            // note: y and z axis flipped:
            x += spnavevent.motion.x;
            y += spnavevent.motion.z;
            z += spnavevent.motion.y;
            rx += -spnavevent.motion.rx;
            ry += -spnavevent.motion.rz;
            rz += -spnavevent.motion.ry;

            speventCount++;
        }
        else
        {
            // SPNAV_EVENT_BUTTON
            static bool button1, button2;
            spin.NodeMessage(spnavNodeID_.c_str(), "ssii", "event", "button", spnavevent.button.bnum, (int)spnavevent.button.press, SPIN_ARGS_END);
            if (spnavevent.button.bnum==0) button1 = (bool)spnavevent.button.press;
            if (spnavevent.button.bnum==1) button2 = (bool)spnavevent.button.press;
            if (button1 && button2)
            {
                spin.NodeMessage(spnavNodeID_.c_str(), "s", "goHome", SPIN_ARGS_END);
            }
        }
    }

    // if at least one update was received this frame
    if (speventCount)
    {
        moving_ = true;

        // make average of motion:
        x /= speventCount;
        y /= speventCount;
        z /= speventCount;
        rx /= speventCount;
        ry /= speventCount;
        rz /= speventCount;

        // if user is pushing beyond a threshold, inrement
        // the speed over time
        // NOTE: pulling up on the puch is harder to get ti to the max than pushing down, so we handle the up/down separately
        float xyMagnitude = sqrt((x*x)+(y*y));
        if ((xyMagnitude>190)||(z<-200)||(z>150))
            speedScaleValue_ += 0.02;
        else
            speedScaleValue_ -= 0.04;

        // max out at 10x of speed scaling
        if (speedScaleValue_ < 1) speedScaleValue_ = 1.0;
        else if (speedScaleValue_ > 10) speedScaleValue_ = 10.0;

        // compute velocity vector:
        speedScaleValue_= 1.0;
        spVel = osg::Vec3( pow(x * VELOCITY_SCALAR, 5) * speedScaleValue_ * 2,
        pow(y * VELOCITY_SCALAR, 5) * speedScaleValue_ * 2,
        pow(z * VELOCITY_SCALAR, 5) * speedScaleValue_);
        // rotate around the z-axis faster than the other two:
        spSpin = osg::Vec3( pow(rx*SPIN_SCALAR,5),
        pow(ry*SPIN_SCALAR,5),
        pow(rz*SPIN_SCALAR,5));

        // apply user-defined scalar values that scale the resulting vectors:
        spVel = osg::componentMultiply(spVel, velocityScalars_);
        spSpin = osg::componentMultiply(spSpin, spinScalars_);

        //std::cout << "spacenav x,y,z="<<x<<","<<y<<","<<z<<" rx,ry,rz="<<rx<<","<<ry<<","<<rz<<" computed speedScale="<<speedScaleValue_<< ", spVel= " << stringify(spVel) << ", spSpin= " << stringify(spSpin) << std::endl;
        spin.NodeMessage(spnavNodeID_.c_str(), "sfff", "setVelocity", spVel.x(), spVel.y(), spVel.z(), SPIN_ARGS_END);
        spin.NodeMessage(spnavNodeID_.c_str(), "sfff", "setSpin", spSpin.x(), spSpin.y(), spSpin.z(), SPIN_ARGS_END);

        lastNavTick_ = osg::Timer::instance()->tick();
    }

#endif
}


//! Update the frames
void CompositeViewer::frame(double f)
{
    // update default viewer
    // this should also update the post processing graph
    // since it is attached to the camera
    //osgViewer::CompositeViewer::frame(f);

    // initilize PPU if it was not done before
    //initializePPU();


    // poll the space navigator:
    updateSpaceNavigator();

    // we used to just call viewer.frame() within a mutex, but we
    // only really need to apply the mutex to the update traversal
    /*
    pthread_mutex_lock(&sceneMutex);
    viewer.frame();
    pthread_mutex_unlock(&sceneMutex);
    */

    // if the userNode's nodepath has changed, we must call setTrackNode
    // to force NodeTrackerManipulator to store the proper nodePath
    if (spinApp::Instance().userNode->nodepathUpdate)
    {
        //std::cout << "nodepathUpdate. " << this->getNumViews() << " views." << std::endl;
        for (unsigned int i=0; i<this->getNumViews(); i++)
        {
            ViewerManipulator *vm = dynamic_cast<ViewerManipulator*>(this->getView(i)->getCameraManipulator());
            if (vm) vm->setTrackNode(spinApp::Instance().userNode->getCameraAttachmentNode());
            //std::cout << "reattached viewer manipulator " << i << " to node: " << spinApp::Instance().userNode->getCameraAttachmentNode()->getName() << std::endl;
        }
        spinApp::Instance().userNode->nodepathUpdate = false;
    }

    this->advance(f);
    this->eventTraversal();
    pthread_mutex_lock(&sceneMutex);
    spinApp::Instance().sceneManager_->update();
    this->updateTraversal();
    this->renderingTraversals();
    pthread_mutex_unlock(&sceneMutex);

}

//int run()



// ----------------------


osg::Geometry* create3DSphericalDisplayDistortionMesh(const osg::Vec3& origin, const osg::Vec3& widthVector, const osg::Vec3& heightVector, double sphere_radius, double collar_radius, double distance)
{

    osg::Vec3d center(0.0,0.0,0.0);
    osg::Vec3d eye(0.0,0.0,0.0);

    // this lets users override the distance:
    if (fabs(distance)<0.000001)
        distance = sqrt(sphere_radius*sphere_radius - collar_radius*collar_radius);

    bool centerProjection = false;
    osg::Vec3d projector = eye - osg::Vec3d(0.0,0.0, distance);

    /*
    osg::notify(osg::NOTICE)<<"Projector position = "<<spin::stringify(projector)<<std::endl;
    osg::notify(osg::NOTICE)<<"distance = "<<distance<<std::endl;
    */

    // create the quad to visualize.
    osg::Geometry* geometry = new osg::Geometry();

    geometry->setSupportsDisplayList(false);

    osg::Vec3 xAxis(widthVector);
    float width = widthVector.length();
    xAxis /= width;

    osg::Vec3 yAxis(heightVector);
    float height = heightVector.length();
    yAxis /= height;

    int noSteps = 50;

    osg::Vec3Array* vertices = new osg::Vec3Array;
    osg::Vec3Array* texcoords0 = new osg::Vec3Array;
    osg::Vec2Array* texcoords1 = new osg::Vec2Array;
    osg::Vec4Array* colors = new osg::Vec4Array;

    osg::Vec3 bottom = origin;
    osg::Vec3 dx = xAxis*(width/((float)(noSteps-1)));
    osg::Vec3 dy = yAxis*(height/((float)(noSteps-1)));

    osg::Vec3d screenCenter = origin + widthVector*0.5f + heightVector*0.5f;
    float screenRadius = heightVector.length() * 0.5f;
    //float screenRadius = heightVector.length() * 0.75f;

    int i,j;
    if (centerProjection)
    {
        for(i=0;i<noSteps;++i)
        {
            osg::Vec3 cursor = bottom+dy*(float)i;
            for(j=0;j<noSteps;++j)
            {
                osg::Vec2 delta(cursor.x() - screenCenter.x(), cursor.y() - screenCenter.y());
                double theta = atan2(-delta.y(), delta.x());
                double phi = osg::PI_2 * delta.length() / screenRadius;
                if (phi > osg::PI_2) phi = osg::PI_2;

                phi *= 2.0;

                // osg::notify(osg::NOTICE)<<"theta = "<<theta<< "phi="<<phi<<std::endl;

                osg::Vec3 texcoord(sin(phi) * cos(theta),
                                   sin(phi) * sin(theta),
                                   cos(phi));

                vertices->push_back(cursor);
                colors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
                texcoords0->push_back(texcoord);
                texcoords1->push_back( osg::Vec2(theta/(2.0*osg::PI), 1.0f - phi/osg::PI_2) );

                cursor += dx;
            }
            // osg::notify(osg::NOTICE)<<std::endl;
        }
    }
    else
    {
        for(i=0;i<noSteps;++i)
        {
            osg::Vec3 cursor = bottom+dy*(float)i;
            for(j=0;j<noSteps;++j)
            {
                osg::Vec2 delta(cursor.x() - screenCenter.x(), cursor.y() - screenCenter.y());
                double theta = atan2(-delta.y(), delta.x());
                double phi = osg::PI_2 * delta.length() / screenRadius;
                if (phi > osg::PI_2) phi = osg::PI_2;

                // osg::notify(osg::NOTICE)<<"theta = "<<theta<< "phi="<<phi<<std::endl;

                double f = distance * sin(phi);
                double e = distance * cos(phi) + sqrt( sphere_radius*sphere_radius - f*f);
                double l = e * cos(phi);
                double h = e * sin(phi);
                double z = l - distance;

                osg::Vec3 texcoord(h * cos(theta) / sphere_radius,
                                   h * sin(theta) / sphere_radius,
                                   z / sphere_radius);

                vertices->push_back(cursor);
                colors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
                texcoords0->push_back(texcoord);
                texcoords1->push_back( osg::Vec2(theta/(2.0*osg::PI), 1.0f - phi/osg::PI_2) );

                cursor += dx;
            }
            // osg::notify(osg::NOTICE)<<std::endl;
        }
    }

    // pass the created vertex array to the points geometry object.
    geometry->setVertexArray(vertices);

    geometry->setColorArray(colors);
    geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

    geometry->setTexCoordArray(0,texcoords0);
    geometry->setTexCoordArray(1,texcoords1);

    for(i=0;i<noSteps-1;++i)
    {
        osg::DrawElementsUShort* elements = new osg::DrawElementsUShort(osg::PrimitiveSet::QUAD_STRIP);
        for(j=0;j<noSteps;++j)
        {
            elements->push_back(j+(i+1)*noSteps);
            elements->push_back(j+(i)*noSteps);
        }
        geometry->addPrimitiveSet(elements);
    }

    return geometry;
}


void makeDomeView(osg::GraphicsContext *gc, osg::GraphicsContext::Traits *traits, osg::View *view, osg::Camera *cam, int textureSize, double radius, double collar, double distance, double crop, osg::Image* intensityMap, const osg::Matrixd& projectorMatrix)
{
    bool applyIntensityMapAsColours = true;

    int camera_width = textureSize;
    int camera_height = textureSize;

    osg::TextureCubeMap* texture = new osg::TextureCubeMap;

    texture->setTextureSize(textureSize, textureSize);
    texture->setInternalFormat(GL_RGB);
    texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR);
    texture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
    texture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
    texture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
    texture->setWrap(osg::Texture::WRAP_R, osg::Texture::CLAMP_TO_EDGE);
    
    /*
    std::cout << "projectorMatrix: " << std::endl;
    std::cout<<projectorMatrix.ptr()[0]<<" "<<projectorMatrix.ptr()[1]<<" "<<projectorMatrix.ptr()[2]<<" "<<projectorMatrix.ptr()[3]<<std::endl;
    std::cout<<projectorMatrix.ptr()[4]<<" "<<projectorMatrix.ptr()[5]<<" "<<projectorMatrix.ptr()[6]<<" "<<projectorMatrix.ptr()[7]<<std::endl;
    std::cout<<projectorMatrix.ptr()[8]<<" "<<projectorMatrix.ptr()[9]<<" "<<projectorMatrix.ptr()[10]<<" "<<projectorMatrix.ptr()[11]<<std::endl;
    std::cout<<projectorMatrix.ptr()[12]<<" "<<projectorMatrix.ptr()[13]<<" "<<projectorMatrix.ptr()[14]<<" "<<projectorMatrix.ptr()[15]<<std::endl;
    */
    

    osg::Camera::RenderTargetImplementation renderTargetImplementation = osg::Camera::FRAME_BUFFER_OBJECT;
    GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;

    unsigned int mipmapLevel = 0;
    bool mipmapGeneration = true;
    unsigned int multisampleSamples = traits->samples;
    unsigned int multisampleColorSamples = traits->samples;
    
    //texture->setNumMipmapLevels(8);

    // front face
    {
        osg::ref_ptr<osg::Camera> camera = new osg::Camera;
        camera->setName("Front face camera");
        camera->setGraphicsContext(gc);
        camera->setViewport(new osg::Viewport(0,0,camera_width, camera_height));
        camera->setDrawBuffer(buffer);
        camera->setReadBuffer(buffer);
        camera->setAllowEventFocus(false);
        // tell the camera to use OpenGL frame buffer object where supported.
        camera->setRenderTargetImplementation(renderTargetImplementation);

        // attach the texture and use it as the color buffer.
        camera->attach(osg::Camera::COLOR_BUFFER, texture, mipmapLevel, osg::TextureCubeMap::POSITIVE_Y, mipmapGeneration, multisampleSamples, multisampleColorSamples);

        view->addSlave(camera.get(), osg::Matrixd(), projectorMatrix * osg::Matrixd());
    }

    // top face
    {
        osg::ref_ptr<osg::Camera> camera = new osg::Camera;
        camera->setName("Top face camera");
        camera->setGraphicsContext(gc);
        camera->setViewport(new osg::Viewport(0,0,camera_width, camera_height));
        //GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
        camera->setDrawBuffer(buffer);
        camera->setReadBuffer(buffer);
        camera->setAllowEventFocus(false);

        // tell the camera to use OpenGL frame buffer object where supported.
        camera->setRenderTargetImplementation(renderTargetImplementation);

        // attach the texture and use it as the color buffer.
        camera->attach(osg::Camera::COLOR_BUFFER, texture, mipmapLevel, osg::TextureCubeMap::POSITIVE_Z, mipmapGeneration, multisampleSamples, multisampleColorSamples);

        view->addSlave(camera.get(), osg::Matrixd(), projectorMatrix * osg::Matrixd::rotate(osg::inDegrees(-90.0f), 1.0,0.0,0.0));
    }

    // left face
    {
        osg::ref_ptr<osg::Camera> camera = new osg::Camera;
        camera->setName("Left face camera");
        camera->setGraphicsContext(gc);
        camera->setViewport(new osg::Viewport(0,0,camera_width, camera_height));
        camera->setDrawBuffer(buffer);
        camera->setReadBuffer(buffer);
        camera->setAllowEventFocus(false);

        // tell the camera to use OpenGL frame buffer object where supported.
        camera->setRenderTargetImplementation(renderTargetImplementation);

        // attach the texture and use it as the color buffer.
        camera->attach(osg::Camera::COLOR_BUFFER, texture, mipmapLevel, osg::TextureCubeMap::NEGATIVE_X, mipmapGeneration, multisampleSamples, multisampleColorSamples);

        view->addSlave(camera.get(), osg::Matrixd(), projectorMatrix * osg::Matrixd::rotate(osg::inDegrees(-90.0f), 0.0,1.0,0.0) * osg::Matrixd::rotate(osg::inDegrees(-90.0f), 0.0,0.0,1.0));
    }

    // right face
    {
        osg::ref_ptr<osg::Camera> camera = new osg::Camera;
        camera->setName("Right face camera");
        camera->setGraphicsContext(gc);
        camera->setViewport(new osg::Viewport(0,0,camera_width, camera_height));
        camera->setDrawBuffer(buffer);
        camera->setReadBuffer(buffer);
        camera->setAllowEventFocus(false);

        // tell the camera to use OpenGL frame buffer object where supported.
        camera->setRenderTargetImplementation(renderTargetImplementation);

        // attach the texture and use it as the color buffer.
        camera->attach(osg::Camera::COLOR_BUFFER, texture, mipmapLevel, osg::TextureCubeMap::POSITIVE_X, mipmapGeneration, multisampleSamples, multisampleColorSamples);

        view->addSlave(camera.get(), osg::Matrixd(), projectorMatrix * osg::Matrixd::rotate(osg::inDegrees(90.0f), 0.0,1.0,0.0 ) * osg::Matrixd::rotate(osg::inDegrees(90.0f), 0.0,0.0,1.0));
    }

    // bottom face
    {
        osg::ref_ptr<osg::Camera> camera = new osg::Camera;
        camera->setGraphicsContext(gc);
        camera->setName("Bottom face camera");
        camera->setViewport(new osg::Viewport(0,0,camera_width, camera_height));
        camera->setDrawBuffer(buffer);
        camera->setReadBuffer(buffer);
        camera->setAllowEventFocus(false);

        // tell the camera to use OpenGL frame buffer object where supported.
        camera->setRenderTargetImplementation(renderTargetImplementation);

        // attach the texture and use it as the color buffer.
        camera->attach(osg::Camera::COLOR_BUFFER, texture, mipmapLevel, osg::TextureCubeMap::NEGATIVE_Z, mipmapGeneration, multisampleSamples, multisampleColorSamples);

        view->addSlave(camera.get(), osg::Matrixd(), projectorMatrix * osg::Matrixd::rotate(osg::inDegrees(90.0f), 1.0,0.0,0.0) * osg::Matrixd::rotate(osg::inDegrees(180.0f), 0.0,0.0,1.0));
    }

    // back face
    {
        osg::ref_ptr<osg::Camera> camera = new osg::Camera;
        camera->setName("Back face camera");
        camera->setGraphicsContext(gc);
        camera->setViewport(new osg::Viewport(0,0,camera_width, camera_height));
        camera->setDrawBuffer(buffer);
        camera->setReadBuffer(buffer);
        camera->setAllowEventFocus(false);

        // tell the camera to use OpenGL frame buffer object where supported.
        camera->setRenderTargetImplementation(renderTargetImplementation);

        // attach the texture and use it as the color buffer.
        camera->attach(osg::Camera::COLOR_BUFFER, texture, mipmapLevel, osg::TextureCubeMap::NEGATIVE_Y, mipmapGeneration, multisampleSamples, multisampleColorSamples);

        view->addSlave(camera.get(), osg::Matrixd(), projectorMatrix * osg::Matrixd::rotate(osg::inDegrees(180.0f), 1.0,0.0,0.0));
    }

    cam->setProjectionMatrixAsPerspective(90.0f, 1.0, 1, 1000.0);

    // distortion correction set up.
    {
        osg::Geode* geode = new osg::Geode();
        // old method:
        //geode->addDrawable(create3DSphericalDisplayDistortionMesh(osg::Vec3(0.0f,0.0f,0.0f), osg::Vec3(traits->width,0.0f,0.0f), osg::Vec3(0.0f,traits->height,0.0f), radius, collar, applyIntensityMapAsColours ? intensityMap : 0, projectorMatrix));
        geode->addDrawable(create3DSphericalDisplayDistortionMesh(osg::Vec3(0.0f,0.0f,0.0f), osg::Vec3(traits->width,0.0f,0.0f), osg::Vec3(0.0f,traits->height,0.0f), radius, collar, distance));

        // new we need to add the texture to the mesh, we do so by creating a
        // StateSet to contain the Texture StateAttribute.
        osg::StateSet* stateset = geode->getOrCreateStateSet();
        stateset->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);
        stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

        if (!applyIntensityMapAsColours && intensityMap)
        {
            stateset->setTextureAttributeAndModes(1, new osg::Texture2D(intensityMap), osg::StateAttribute::ON);
        }

        // mikewoz: we pass the pre-made camera here:
        osg::ref_ptr<osg::Camera> camera = new osg::Camera;
        //osg::ref_ptr<osg::Camera> camera = cam;

        int stretch_x = traits->width * 4 / 3 - traits->width;

        camera->setGraphicsContext(gc);
        camera->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT );
        camera->setClearColor( osg::Vec4(0.0,0.0,0.0,1.0) );
        camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
        //camera->setViewport(new osg::Viewport(-stretch_x / 2, 0, traits->width + stretch_x, traits->height));
        GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
        camera->setDrawBuffer(buffer);
        camera->setReadBuffer(buffer);
        camera->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
        camera->setAllowEventFocus(false);
        camera->setInheritanceMask(camera->getInheritanceMask() & ~osg::CullSettings::CLEAR_COLOR & ~osg::CullSettings::COMPUTE_NEAR_FAR_MODE);
        //camera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);

        //int crop = 125;
        //int crop = 132;
        //int crop = 50;
        //int crop = 0;
        double aspect = traits->width / traits->height;
        camera->setProjectionMatrixAsOrtho2D( crop*aspect, traits->width-(crop*aspect), crop, traits->height-crop);
        //camera->setProjectionMatrixAsOrtho2D(0,traits->width,0,traits->height);


        camera->setViewMatrix(osg::Matrix::identity());

        // add subgraph to render
        camera->addChild(geode);

        camera->setName("DistortionCorrectionCamera");

        view->addSlave(camera.get(), osg::Matrixd(), osg::Matrixd(), false);
    }

    cam->setNearFarRatio(0.0001f);
    
    // In the spherical case, all cameras are slaves, but we couldn't figure
    // out how to get rid of the original cam so we just set it's viewport to
    // zero. If you don't do this, then this camera and the distortion camera
    // will z-fight and you'll see flickering.
    cam->setViewport(0,0,0,0);
}



void loadXMLcamera(TiXmlElement *XMLnode, osgViewer::Viewer::View *view, osg::Camera *cam, osg::GraphicsContext::Traits *traits, osg::GraphicsContext *gc)
{
    TiXmlElement *child = 0;
    std::string tag="", val="";
    float v[4];

    bool spherical = false;

    //osg::Vec3 eye = osg::Vec3(0,0,0);
    //osg::Vec3 lookat = osg::Y_AXIS;
    //osg::Vec3 up = osg::Z_AXIS;
    osg::Vec3 eye = osg::Vec3(0,-0.00000001,0);
    osg::Vec3 lookat = osg::Vec3(0,0,0);
    osg::Vec3 up = osg::Z_AXIS;

    if (XMLnode->Attribute("id"))
    {
        std::cout << "    Loading camera: " << XMLnode->Attribute("id") << " (" << traits->width << "x" << traits->height << ")" << std::endl;
        cam->setName(XMLnode->Attribute("id"));
    }
    else std::cout << "    Loading camera (" << traits->width << "x" << traits->height << ")" << std::endl;


    // FIRST PASS: parse generic parameters, and check if spherical cam
    for ( child = XMLnode->FirstChildElement(); child; child = child->NextSiblingElement() )
    {
        // get tag and value:
        if (child->FirstChild())
        {
            tag = child->Value();
            val = child->FirstChild()->Value();
        } else continue;

        if (tag=="clearColor")
        {
            if (sscanf (val.c_str(),"%f %f %f %f",&v[0],&v[1],&v[2],&v[3]))
                cam->setClearColor( osg::Vec4(v[0],v[1],v[2],v[3]) );
        }
        else if (tag=="viewport")
        {
            if (sscanf(val.c_str(),"%f%% %f%% %f%% %f%%",&v[0],&v[1],&v[2],&v[3])==4)
            {
                //cam->setViewport( (int) (v[0]/100*screenWidth/100), (int) (v[1]/100*screenHeight), (int) (v[2]/100*screenWidth), (int) (v[3]/100*screenHeight) );
                cam->setViewport( (int) (v[0]/100*traits->width/100), (int) (v[1]/100*traits->height), (int) (v[2]/100*traits->width), (int) (v[3]/100*traits->height) );
            }
            else if (sscanf(val.c_str(),"%f %f %f %f",&v[0],&v[1],&v[2],&v[3])==4)
            {
                //view->getCamera()->setViewport( v[0],v[1],v[2],v[3] );
                cam->setViewport( v[0],v[1],v[2],v[3] );
            }
            else {
                std::cout << "Bad viewport values: " << val << ". Need four values <x y width height>, either as pixel values of percentages of the window size" << std::endl;
            }

            const osg::Viewport *viewport = cam->getViewport();
            if (viewport)
                std::cout << "     Camera viewport:  pos=(" << viewport->x() << "," << viewport->y() << ") size=" << viewport->width() << "x" << viewport->height() << std::endl;
            else
                std::cout << "     Camera viewport:  INVALID" << std::endl;

        }
        else if (tag=="spherical")
        {
            std::cout << "   (spherical camera)" << std::endl;
            spherical = true;
        }
    }


    // SECOND PASS: different options depending on spherical vs planar camera:

    if (spherical)
    {
        int textureSize = 512;
        float radius = 1.0;
        float collar = 0.45;
        float distance = 0.0;
        float rotation = 0.0;
        float crop = 0.0;
        float near = 1.0;
        float far = 1000.0;
        osg::Matrixd projMatrix = osg::Matrixd::identity();

        for ( child = XMLnode->FirstChildElement(); child; child = child->NextSiblingElement() )
        {
            if (child->FirstChild())
            {
                tag = child->Value();
                val = child->FirstChild()->Value();
            } else continue;

            if (tag=="spherical")
            {
                // noop
            }
            else if (tag=="textureSize")
            {
                sscanf(val.c_str(), "%d", &textureSize);
            }
            else if (tag=="radius")
            {
                sscanf(val.c_str(), "%f", &radius);
            }
            else if (tag=="collar")
            {
                sscanf(val.c_str(), "%f", &collar);
            }
            else if (tag=="distance")
            {
                sscanf(val.c_str(), "%f", &distance);
            }
            else if (tag=="crop")
            {
                sscanf(val.c_str(), "%f", &crop);
            }
            else if (tag=="rotation")
            {
                sscanf(val.c_str(), "%f", &rotation);
            }
            else if (tag=="clipping")
            {
                sscanf(val.c_str(), "%f %f", &near, &far);
                std::cout << "got clipping for spherical cam: " << near << " " << far << std::endl;
            }
            else
            {
                std::cout << "WARNING: unrecognized option for spherical camera: " << tag << " " << val << std::endl;
            }
        }

        // TODO: read image paths from config for intensity map
        osg::Image *intensityMap = 0;
        
        // rotation around X_AXIS. by default, a spherical display points up.
        // A rotation of 180 would point down.
        projMatrix *= osg::Matrixd::rotate(osg::inDegrees(rotation), osg::X_AXIS);
        
        std::cout << "creating spherical display with textureSize="<<textureSize<<", multiSamples="<<traits->samples<<", radius="<<radius<<", collar="<<collar<<", crop="<<crop<<", rotation="<<rotation<<"degrees"<<std::endl;

        // note: cam is currently not being used by makeDomeView!
        makeDomeView(gc, traits, view, cam, textureSize, radius, collar, distance, crop, intensityMap, projMatrix);

        double fovy, aspectRatio, zNear, zFar;
        cam->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
        cam->getProjectionMatrixAsPerspective(fovy, aspectRatio, zNear, zFar);
        cam->setProjectionMatrixAsPerspective(fovy, aspectRatio, near, far);
    }

    // planar:
    else
    {
        for ( child = XMLnode->FirstChildElement(); child; child = child->NextSiblingElement() )
        {
            if (child->FirstChild())
            {
                tag = child->Value();
                val = child->FirstChild()->Value();
            } else continue;

            if (tag=="eye")
            {
                if (sscanf (val.c_str(),"%f %f %f",&v[0],&v[1],&v[2]))
                    eye = osg::Vec3(v[0],v[1],v[2]);
            }
            else if (tag=="lookat")
            {
                if (sscanf (val.c_str(),"%f %f %f",&v[0],&v[1],&v[2]))
                    lookat = osg::Vec3(v[0],v[1],v[2]);
            }
            else if (tag=="up")
            {
                if (sscanf (val.c_str(),"%f %f %f",&v[0],&v[1],&v[2]))
                    up = osg::Vec3(v[0],v[1],v[2]);
            }
            else if (tag == "perspective")
            {
                float fovy, aspectRatio, zNear, zFar;
                if (sscanf(val.c_str(), "%f %f %f %f", &fovy, &aspectRatio, &zNear, &zFar))
                {
                    cam->setProjectionMatrixAsPerspective(fovy, aspectRatio, zNear, zFar);
                }
            }
            else if (tag=="frustum")
            {
                frustum frust;
                if (sscanf (val.c_str(),"%f %f %f %f %f %f",&frust.left,&frust.right,&frust.bottom,&frust.top,&frust.near,&frust.far))
                {
                    cam->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
                    cam->setProjectionMatrixAsFrustum(frust.left, frust.right, frust.bottom, frust.top, frust.near, frust.far);
                }
            }

            else
            {
                std::cout << "Unknown parameter in configuration file: " << tag << std::endl;
            }
        }
    }

    ViewerManipulator *manipulator = new spin::ViewerManipulator();
    manipulator->setHomePosition( eye, lookat, up, false );
    view->setCameraManipulator(manipulator);

    GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
    cam->setDrawBuffer(buffer);
    cam->setReadBuffer(buffer);

}




void loadXMLwindow(TiXmlElement *XMLnode, osgViewer::CompositeViewer &viewer)
{
    TiXmlElement *n;

    // first check if the wsi is valid:
    osg::GraphicsContext::WindowingSystemInterface* wsi = osg::GraphicsContext::getWindowingSystemInterface();
    if (!wsi)
    {
        osg::notify(osg::NOTICE)<<"Error, no WindowSystemInterface available, cannot create windows."<<std::endl;
        return;
    }


    osg::ref_ptr<osgViewer::View> view = new osgViewer::View;
    if (XMLnode->Attribute("id"))
    {
        std::cout << "  Loading window: " << XMLnode->Attribute("id") << std::endl;
        view->setName("SPIN Viewer: "+std::string(XMLnode->Attribute("id")));
    }
    else std::cout << "  Loading window" << std::endl;

    osg::DisplaySettings* ds;
    if (view->getDisplaySettings())
    {
        ds = view->getDisplaySettings();
     }
    else
    {
        //std::cout << "Display settings not valid" << std::endl;
        ds = osg::DisplaySettings::instance();
    }


    osg::GraphicsContext::ScreenIdentifier si;
    si.readDISPLAY();

    // displayNum has not been set so reset it to 0.
    if (si.displayNum<0) si.displayNum = 0;

    // get screenNum from config file, or default to 0:
    if ((n = XMLnode->FirstChildElement("screen")))
        si.screenNum = atoi(n->FirstChild()->Value());
    else
        si.screenNum = 0;


    // Now that we have the screen, let's get the resolution. This is important
    // because size and position in the xml file can be specified as a
    // percentage
    unsigned int screenWidth, screenHeight;
    wsi->getScreenResolution(osg::GraphicsContext::ScreenIdentifier(si.screenNum), screenWidth, screenHeight);
    std::cout << "Resolution for screen " << si.screenNum << " is: " << screenWidth << "x" << screenHeight << std::endl;

    /*
    if (n = XMLnode->FirstChildElement("fullscreen"))
    {
        if (n->FirstChild()->Value() == "true")
        {
            view->setUpViewOnSingleScreen(screenNum);
        }
        else
        {
            int x=50;
            int y=50;
            int w=800;
            int h=600;
            if (n = XMLnode->FirstChildElement("windowPosition"))
                sscanf( n->FirstChild()->Value(), "%d %d", &x, &y);
            if (n = XMLnode->FirstChildElement("windowSize"))
                sscanf( n->FirstChild()->Value(), "%d %d", &w, &h);
            view->setUpViewInWindow(x,y,w,h,screenNum);
            maxWidth = w;
            maxHeight = h;
        }
    }
    */


    // create a GraphicsContext::Traits for this window and initialize with
    // some defaults:
    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits(osg::DisplaySettings::instance().get());

    //traits->hostName = si.hostName;
    traits->displayNum = 0;//si.displayNum;
    traits->screenNum = si.screenNum;
    traits->x = 0;
    traits->y = 0;
    traits->width = 320;
    traits->height = 240;
    traits->windowDecoration = true;
    traits->doubleBuffer = true;
    traits->useCursor = true;
    traits->supportsResize = true;
    traits->sharedContext = 0;
    traits->windowName = view->getName();

    // update window position based on config file:
    if ((n = XMLnode->FirstChildElement("windowPosition")))
    {
        float percentWidth, percentHeight;
        if (sscanf( n->FirstChild()->Value(), "%f%% %f%%", &percentWidth, &percentHeight)==2)
        {
            traits->x = (int) screenWidth * (percentWidth/100);
            traits->y = (int) screenHeight * (percentHeight/100);
        }
        else
        {
            sscanf( n->FirstChild()->Value(), "%d %d", &traits->x, &traits->y);
        }
    }

    if ((n = XMLnode->FirstChildElement("windowSize")))
    {
        float percentWidth, percentHeight;
        if (sscanf( n->FirstChild()->Value(), "%f%% %f%%", &percentWidth, &percentHeight)==2)
        {
            traits->width = (int) screenWidth * (percentWidth/100);
            traits->height = (int) screenHeight * (percentHeight/100);
        }
        else
        {
            sscanf( n->FirstChild()->Value(), "%d %d", &traits->width, &traits->height);
        }
    }

    if ((n = XMLnode->FirstChildElement("supportsResize")))
    {
        if (osgDB::convertToLowerCase(n->FirstChild()->Value()).compare(std::string("false")) == 0)
        {
            traits->supportsResize = false;
        }
        else
        {
            traits->supportsResize = true;
            view->addEventHandler(new osgViewer::WindowSizeHandler);
        }
    }

    if ((n = XMLnode->FirstChildElement("useCursor")))
    {
        if (osgDB::convertToLowerCase(n->FirstChild()->Value()).compare(std::string("false")) == 0)
            traits->useCursor = false;
        else
            traits->useCursor = true;
    }

    if ((n = XMLnode->FirstChildElement("windowDecoration")))
    {
        if (osgDB::convertToLowerCase(n->FirstChild()->Value()).compare(std::string("false")) == 0)
            traits->windowDecoration = false;
        else
            traits->windowDecoration = true;
    }

    if ((n = XMLnode->FirstChildElement("multiSamples")))
    {
        int numSamples;
        if (sscanf( n->FirstChild()->Value(), "%d", &numSamples)==1)
        {
            traits->samples = numSamples;
            //traits->sampleBuffers = numSamples;
        }
    }

    osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());

    // now search for cameras:
    bool firstCamera = true;
    for ( n = XMLnode->FirstChildElement("camera"); n; n = n->NextSiblingElement("camera") )
    {
        osg::Camera *cam;
        if (firstCamera)
        {
            cam = view->getCamera();
            firstCamera = false;
        }
        else
        {
            cam = new osg::Camera();
            view->addSlave(cam, view->getCamera()->getProjectionMatrix(), view->getCamera()->getViewMatrix());
        }
        if (gc.valid()) cam->setGraphicsContext(gc.get());
        else std::cout << "ERROR: GraphicsContext not valid. Bad configuration file?" << std::endl;

        // Projection matrix aspect fix (can be overridden using either the
        // frustum or perspective configuration values in config file)
        if (0) {
            double fovy, aspectRatio, zNear, zFar;
            cam->getProjectionMatrixAsPerspective(fovy, aspectRatio, zNear, zFar);

            double newAspectRatio = double(traits->width) / double(traits->height);
            double aspectRatioChange = newAspectRatio / aspectRatio;
            if (aspectRatioChange != 1.0)
            {
                cam->getProjectionMatrix() *= osg::Matrix::scale(1.0/aspectRatioChange,1.0,1.0);
            }
        }

        //loadXMLcamera( n, view, cam, traits->width, traits->height, traits->screenNum);
        loadXMLcamera( n, view, cam, traits, gc);

        /*
        GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
        cam->setDrawBuffer(buffer);
        cam->setReadBuffer(buffer);
        */
    }

    view->setLightingMode(osg::View::SKY_LIGHT);
    view->addEventHandler(new osgViewer::StatsHandler);
    view->setSceneData(spinApp::Instance().sceneManager_->rootNode.get());
    viewer.addView(view.get());

    osgViewer::GraphicsWindow* gw = dynamic_cast<osgViewer::GraphicsWindow*>(gc.get());
    if (gw)
    {
        gw->getEventQueue()->getCurrentEventState()->setWindowRectangle(0, 0, traits->width, traits->height );
    }
}


int viewerCallback(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data)
{
    spin::spinApp &spin = spin::spinApp::Instance();
    CompositeViewer *viewer = (CompositeViewer*) user_data;

    if (0)
    {
        printf("************ viewer got message: %s\n", (char*)path);
        for (int i = 0; i < argc; i++)
        {
            printf("arg %d '%c' ", i, types[i]);
            lo_arg_pp((lo_type) types[i], argv[i]);
            printf("\n");
        }
        printf("\n");
        fflush(stdout);
    }


    // make sure there is at least one argument (ie, a method to call):
    if (!argc) return 1;

    // get the method (argv[0]):
    std::string theMethod;
    if (lo_is_string_type((lo_type)types[0]))
    {
        theMethod = std::string((char *)argv[0]);
    }
    else return 1;

    // parse the rest of the args:
    std::vector<float> floatArgs;
    std::vector<std::string> stringArgs;
    for (int i=1; i<argc; i++)
    {
        if (lo_is_numerical_type((lo_type)types[i]))
        {
            floatArgs.push_back( (float) lo_hires_val((lo_type)types[i], argv[i]) );
        } else {
            stringArgs.push_back( (const char*) argv[i] );
        }
    }

    bool lIsDof, lIsSSAO, lIsMBlur, lIsOutline, lIsMask, lIsShader;

    int lNumPPUs = 0;
    if(!viewer->isDome())
    {
        lNumPPUs = viewer->getNumViews();
        lIsDof = (viewer->mDofPPUs.size() == lNumPPUs);
        lIsSSAO = (viewer->mSsaoPPUs.size() == lNumPPUs);
        lIsMBlur = (viewer->mMBlurPPUs.size() == lNumPPUs);
        lIsOutline = (viewer->mOutlinePPUs.size() == lNumPPUs);
        lIsMask = (viewer->mMaskPPUs.size() == lNumPPUs);
        lIsShader = (viewer->mShaderPPUs.size() == lNumPPUs);
    }
    else
    {
        // Dome view, we should have 7 slave cameras, 6 with ppus
        // but we should check that anyway
        for(int i=0; i<viewer->getView(0)->getNumSlaves(); i++)
            if(viewer->getView(0)->getSlave(i)._camera.get()->getRenderTargetImplementation() == osg::Camera::FRAME_BUFFER_OBJECT)
                lNumPPUs++;

        lIsDof = (viewer->mDofPPUs.size() == lNumPPUs);
        lIsSSAO = (viewer->mSsaoPPUs.size() == lNumPPUs);
        lIsMBlur = (viewer->mMBlurPPUs.size() == lNumPPUs);
        lIsOutline = (viewer->mOutlinePPUs.size() == lNumPPUs);
        lIsMask = (viewer->mMaskPPUs.size() == lNumPPUs);
        lIsShader = (viewer->mShaderPPUs.size() == lNumPPUs);
    }

    if ((theMethod=="setParam") && (stringArgs.size()==1) && (floatArgs.size()==1))
    {
        // For each view
        for(unsigned int i=0; i<lNumPPUs; ++i)
        {
            // Params for the DoF PPU
            if(lIsDof)
            {
                if (stringArgs[0] == "gaussSigma")
                {
                    viewer->mDofPPUs[i]->setGaussSigma(floatArgs[0]);
                }
                else if (stringArgs[0] == "gaussRadius")
                {
                    viewer->mDofPPUs[i]->setGaussRadius(floatArgs[0]);
                }
                else if (stringArgs[0] == "focalLength")
                {
                    viewer->mDofPPUs[i]->setFocalLength(floatArgs[0]);
                }
                else if (stringArgs[0] == "focalRange")
                {
                    viewer->mDofPPUs[i]->setFocalRange(floatArgs[0]);
                }
                else if (stringArgs[0] == "near")
                {
                    viewer->mDofPPUs[i]->setNear(floatArgs[0]);
                }
                else if (stringArgs[0] == "far")
                {
                    viewer->mDofPPUs[i]->setFar(floatArgs[0]);
                }
            }

            // Params for the SSAO PPU
            if(lIsSSAO)
            {
                if (stringArgs[0] == "ssaoGaussSigma")
                {
                    viewer->mSsaoPPUs[i]->setGaussSigma(floatArgs[0]);
                }
                else if (stringArgs[0] == "ssaoGaussRadius")
                {
                    viewer->mSsaoPPUs[i]->setGaussRadius(floatArgs[0]);
                }
                else if (stringArgs[0] == "ssaoPower")
                {
                    viewer->mSsaoPPUs[i]->setSsaoPower(floatArgs[0]);
                }
                else if (stringArgs[0] == "ssaoFocus")
                {
                    viewer->mSsaoPPUs[i]->setSsaoFocus(floatArgs[0]);
                }
                else if (stringArgs[0] == "ssaoSamples")
                {
                    viewer->mSsaoPPUs[i]->setSsaoSamples((int)floatArgs[0]);
                }
                else if (stringArgs[0] == "ssaoResample")
                {
                    viewer->mSsaoPPUs[i]->setResampleFactor(floatArgs[0]);
                }
            }

            // Params for the motion blur PPU
            if(lIsMBlur)
            {
                if (stringArgs[0] == "motionBlurFactor")
                {
                    viewer->mMBlurPPUs[i]->setMotionBlurFactor(floatArgs[0]);
                }
            }

            // Params for the outline PPU
            if(lIsOutline)
            {
                if (stringArgs[0] == "outlineStrength")
                {
                    viewer->mOutlinePPUs[i]->setOutlineStrength(floatArgs[0]);
                }
                else if (stringArgs[0] == "outlineGlowSigma")
                {
                    viewer->mOutlinePPUs[i]->setGlowSigma(floatArgs[0]);
                }
                else if (stringArgs[0] == "outlineGlowRadius")
                {
                    viewer->mOutlinePPUs[i]->setGlowRadius(floatArgs[0]);
                }
                else if (stringArgs[0] == "outlineGlowPower")
                {
                    viewer->mOutlinePPUs[i]->setGlowPower(floatArgs[0]);
                }
                else if (stringArgs[0] == "outlineMode")
                {
                    viewer->mOutlinePPUs[i]->setOutlineMode((int)floatArgs[0]);
                }
                else if (stringArgs[0] == "outlineResample")
                {
                    viewer->mOutlinePPUs[i]->setResampleFactor(floatArgs[0]);
                }
            }

            // Params for the mask PPU
            if(lIsMask)
            {
                if (stringArgs[0] == "maskTransparency")
                {
                    viewer->mMaskPPUs[i]->setMaskTransparency(floatArgs[0]);
                }
                else if (stringArgs[0] == "maskLightingDistance")
                {
                    viewer->mMaskPPUs[i]->setMaskLightingDistance(floatArgs[0]);
                }
                else if (stringArgs[0] == "maskLightSearchDistance")
                {
                    viewer->mMaskPPUs[i]->setLightSearchDistance(floatArgs[0]);
                }
                else if (stringArgs[0] == "maskLightSearchStep")
                {
                    viewer->mMaskPPUs[i]->setLightSearchStep(floatArgs[0]);
                }
            }

        }


        return 1;
    }
    else if ((theMethod=="setParam") && (stringArgs.size()==2))
    {
        // For each view
        for(unsigned int i=0; i<lNumPPUs; ++i)
        {
            // Params for the custom shader PPU
            if(lIsShader)
            {
                if (stringArgs[0] == "shader")
                {
                    viewer->mShaderPPUs[i]->setShaderFile(stringArgs[1]);
                }
            }
        }
    }
    else if ((theMethod=="setOutlineColor") && (floatArgs.size()==4))
    {
        // For each view
        for(unsigned int i=0; i<lNumPPUs; ++i)
        {
            if(lIsOutline)
            {
                viewer->mOutlinePPUs[i]->setOutlineColor(floatArgs[0], floatArgs[1], floatArgs[2], floatArgs[3]);
            }
        }
        return 1;
    }
    else if ((theMethod=="setShaderColor") && (stringArgs.size()==1) && (floatArgs.size()==4))
    {
        for (unsigned int i=0; i<lNumPPUs; ++i)
        {
            if (lIsShader)
            {
                viewer->mShaderPPUs[i]->setShaderColor(stringArgs[0], floatArgs[0], floatArgs[1], floatArgs[2], floatArgs[3]);
            }
        }
        return 1;
    }
    else if ((theMethod=="setFrustum") && (floatArgs.size()==6))
    {
        // For each view
        for(unsigned int i=0; i<lNumPPUs; ++i)
        {
            viewer->getView(i)->getCamera()->setProjectionMatrixAsFrustum(floatArgs[0], floatArgs[1], floatArgs[2], floatArgs[3], floatArgs[4], floatArgs[5]);

            if(lIsDof)
            {
                viewer->mDofPPUs[i]->setNear(floatArgs[4]);
                viewer->mDofPPUs[i]->setFar(floatArgs[5]);
            }

            if(lIsSSAO)
            {
                viewer->mSsaoPPUs[i]->setProjectionMatrix(viewer->getView(i)->getCamera()->getProjectionMatrix());
            }
        }

        return 1;
    }
    else if ((theMethod=="setVelocityScalars") && (floatArgs.size()==3))
    {
        viewer->setVelocityScalars(osg::Vec3(floatArgs[0], floatArgs[1], floatArgs[2]));
        return 1;
    }
    else if ((theMethod=="setSpinScalars") && (floatArgs.size()==3))
    {
        viewer->setSpinScalars(osg::Vec3(floatArgs[0], floatArgs[1], floatArgs[2]));
        return 1;
    }
    else if ((theMethod=="setSpaceNavigatorNode") && (stringArgs.size()==1))
    {
        viewer->setSpaceNavigatorNode(stringArgs[0]);
        return 1;
    }

    return 1;
}



} // end namespace
