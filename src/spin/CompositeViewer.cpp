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
#include <boost/filesystem.hpp>


#include <osg/TextureCubeMap>

#include <osgGA/NodeTrackerManipulator>
#include <osgGA/TrackballManipulator>
#include <osgGA/StateSetManipulator>

#include <osgDB/ReadFile>

#include <boost/algorithm/string.hpp>

#include "config.h"
#include "ViewerManipulator.h"
#include "spinUtil.h"
#include "spinApp.h"
#include "spinClientContext.h"
#include "osgUtil.h"
#include "GroupNode.h"
#include "SceneManager.h"
#include "ShapeNode.h"

#include "CompositeViewer.h"

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
    mOldTime = 0.0f;
    lastNavTick_ = osg::Timer::instance()->tick();
    velocityScalars_ = osg::Vec3(1,1,1);
    spinScalars_ = osg::Vec3(1,1,1);
}

osg::Texture* CompositeViewer::createRenderTexture(int tex_width, int tex_height, bool depth)
{
    // create simple 2D texture
    osg::Texture2D* texture2D = new osg::Texture2D;
    texture2D->setTextureSize(tex_width, tex_height);
    texture2D->setResizeNonPowerOfTwoHint(false);
    texture2D->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR);
    texture2D->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
    texture2D->setWrap(osg::Texture2D::WRAP_S,osg::Texture2D::CLAMP_TO_BORDER);
    texture2D->setWrap(osg::Texture2D::WRAP_T,osg::Texture2D::CLAMP_TO_BORDER);
    texture2D->setBorderColor(osg::Vec4(1.0f,1.0f,1.0f,1.0f));

    // setup float format
    if (!depth)
    {
        texture2D->setInternalFormat(GL_RGBA16F_ARB);
        texture2D->setSourceFormat(GL_RGBA);
        texture2D->setSourceType(GL_FLOAT);
        /*
        texture2D->setInternalFormat(GL_RGBA32I_EXT);
        texture2D->setSourceFormat(GL_RGBA_INTEGER_EXT);
        texture2D->setSourceType(GL_BYTE);
        */
    }
    else{
        texture2D->setInternalFormat(GL_DEPTH_COMPONENT);
    }

    return texture2D;
}

void CompositeViewer::setupCamera(osg::Viewport* vp)
{
    // setup viewer's default camera
    //osg::Camera* camera = getCamera();
    camera = this->getView(0)->getCamera();
    
    //camera->setViewport(0,0,(int)vp->width(), (int)vp->height()); 
    
    // TODO: setup for all cameras:
    /*
    osgViewer::Viewer::Cameras cameras;
    viewer.getCameras(cameras);
    for (osgViewer::Viewer::Cameras::iterator iter = cameras.begin(); iter != cameras.end(); ++iter)
    {
        //(*iter)->setNearFarRatio(0.0001f);
        
    }
    */
    
    std::cout << std::endl;
    std::cout << "---------------------------------------" << std::endl;
    std::cout << "setting up camera with wxh=" << (int)vp->width()<<"x"<<(int)vp->height() << std::endl;
    

    // create texture to render to
    colorTexture1_ = createRenderTexture((int)vp->width(), (int)vp->height(), false);
    colorTexture2_ = createRenderTexture((int)vp->width(), (int)vp->height(), false);
    colorTexture3_ = createRenderTexture((int)vp->width(), (int)vp->height(), false);
    depthTexture_ = createRenderTexture((int)vp->width(), (int)vp->height(), true);

    // set up the background color and clear mask.
    camera->setClearColor(osg::Vec4(0.0f,0.0f,0.0f,0.0f));
    camera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // set viewport
    camera->setViewport(vp);
    camera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
    //camera->setProjectionMatrixAsPerspective(20.0, vp->width()/vp->height(), 0.1, 100.0); 

    // tell the camera to use OpenGL frame buffer object where supported.
    camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);

    // attach the texture and use it as the color buffer.
    camera->attach(osg::Camera::COLOR_BUFFER0, colorTexture1_);
    camera->attach(osg::Camera::COLOR_BUFFER1, colorTexture2_);
    camera->attach(osg::Camera::COLOR_BUFFER2, colorTexture3_);
    camera->attach(osg::Camera::DEPTH_BUFFER, depthTexture_);
}

//! Just setup some stuff
void CompositeViewer::viewerInit()
{
    // propagate the method
    osgViewer::CompositeViewer::viewerInit();
    //osgViewer::Viewer::viewerInit();

    // setup data
    //setupCamera(getCamera()->getViewport());
    setupCamera(this->getView(0)->getCamera()->getViewport());
    
    // add ppu processor into the scene graph
    osg::Group* group = new osg::Group();
    //group->addChild(getSceneData());
    group->addChild(this->getView(0)->getSceneData());
    
    //setSceneData(group);
    this->getView(0)->setSceneData(group);

}

//! Setup osgppu for rendering
void CompositeViewer::initializePPU(unsigned int pEffect)
{
    // if already initialized then just do nothing
    if (mbInitialized == false)
        mbInitialized = true;
    else
        return;
    
    if(pEffect == noEffect)
        return;

    mProcessor = new osgPPU::Processor();
    //dynamic_cast<osg::Group*>(getSceneData())->addChild(mProcessor.get());
    dynamic_cast<osg::Group*>(getView(0)->getSceneData())->addChild(mProcessor.get());

    // initialize the post process
    //mProcessor->setCamera(getCamera());
    mProcessor->setCamera(this->getView(0)->getCamera());
    mProcessor->setName("Processor");
    mProcessor->dirtyUnitSubgraph();

    // we want to simulate hdr rendering, hence setup the pipeline
    // for the hdr rendering
    osgPPU::Unit* lastUnit = NULL;
 


    if(pEffect == dofEffect)
    {
        //osg::setNotifyLevel(osg::DEBUG_FP);
        dofPPU_ = new DoFRendering();

        double left,right,bottom,top,near,far; 
        this->getView(0)->getCamera()->getProjectionMatrixAsFrustum(left,right,bottom,top,near,far);

        //dofPPU_->createDoFPipeline(mProcessor.get(), lastUnit, 0.01, 100.0);
        dofPPU_->createDoFPipeline(mProcessor.get(), lastUnit, near, far);
        dofPPU_->setFocalLength(0.0);
        dofPPU_->setFocalRange(50.0);
        //osg::setNotifyLevel(osg::FATAL);
    }
    else if(pEffect == ssaoEffect)
    {   
        ssaoPPU_ = new SSAORendering();
        // Gets the projection matrix
        osg::Matrixf lProjectionMatrix = camera->getProjectionMatrix();

        ssaoPPU_->createSSAOPipeline(mProcessor.get(), lastUnit,
                                colorTexture1_, colorTexture2_, colorTexture3_,
                                lProjectionMatrix);
        ssaoPPU_->setPower(1.f);
    }


    // add a text ppu after the pipeline is setted up
    if (0)
    {
        osgPPU::UnitText* fpstext = new osgPPU::UnitText();
        fpstext->setName("FPSTextPPU");
        fpstext->setSize(44);
        fpstext->setText("Example DoF-pipeline from a .ppu file");
        fpstext->setPosition(0.01, 0.95);
        lastUnit->addChild(fpstext);
    }

    // As a last step we setup a ppu which do render the content of the result
    // on the screenbuffer. This ppu MUST be as one of the last, otherwise you
    // will not be able to get results from the ppu pipeline
    osgPPU::UnitOut* ppuout = new osgPPU::UnitOut();
    ppuout->setName("PipelineResult");
    ppuout->setInputTextureIndexForViewportReference(-1); // need this here to get viewport from camera
    lastUnit->addChild(ppuout);

    // write pipeline to a file
    //osgDB::writeObjectFile(*mProcessor, "dof.ppu");
}

void CompositeViewer::updateSpaceNavigator()
{
#ifdef HAVE_SPNAV_H
    
    spin::spinApp &spin = spin::spinApp::Instance();

    // poll the space navigator:
    int speventCount = 0;
    osg::Vec3 spVel, spSpin;

    //std::cout << "delta = " << osg::Timer::instance()->delta_s(lastNavTick_, osg::Timer::instance()->tick()) << std::endl;

    // if the last significant event was more than a second ago,
    // assume the user has let go of the puck and reset speedScale
    if ((float)(osg::Timer::instance()->delta_s(lastNavTick_, osg::Timer::instance()->tick()) > 1.0) && moving_)
    {
        //std::cout << "reset spacenavigator" << std::endl;
        spin.NodeMessage(spin.getUserID().c_str(), "sfff", "setVelocity", 0.0, 0.0, 0.0, SPIN_ARGS_END);
        spin.NodeMessage(spin.getUserID().c_str(), "sfff", "setSpin", 0.0, 0.0, 0.0, SPIN_ARGS_END);
        speedScaleValue_ = 1.0;
        moving_ = false;
    }
    
    spnav_event spnavevent;
    while (spnav_poll_event(&spnavevent))
    {
        if (spnavevent.type == SPNAV_EVENT_MOTION)
        {
            // just make note of the count; the latest motion data will be stored in spnavevent
            
            speventCount++;
            //spPos = osg::Vec3(spnavevent.motion.x, spnavevent.motion.z, spnavevent.motion.y);
            //spRot = osg::Vec3(spnavevent.motion.rx, spnavevent.motion.rz, spnavevent.motion.ry);
        }
        else
        {
            // SPNAV_EVENT_BUTTON
            static bool button1, button2;
            spin.NodeMessage(spin.getUserID().c_str(), "ssii", "event", "button", spnavevent.button.bnum, (int)spnavevent.button.press, SPIN_ARGS_END);
            if (spnavevent.button.bnum==0) button1 = (bool)spnavevent.button.press;
            if (spnavevent.button.bnum==1) button2 = (bool)spnavevent.button.press;
            if (button1 && button2)
            {
                spin.NodeMessage(spin.getUserID().c_str(), "s", "goHome", SPIN_ARGS_END);
            }
        }
    }

    // if at least one update was received this frame
    if (speventCount)
    {
        moving_ = true;

        // note: y and z axis flipped:
        float x = spnavevent.motion.x;
        float y = spnavevent.motion.z;
        float z = spnavevent.motion.y;
        float rx = -spnavevent.motion.rx;
        float ry = -spnavevent.motion.rz;
        float rz = -spnavevent.motion.ry;

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
        spin.NodeMessage(spin.getUserID().c_str(), "sfff", "setVelocity", spVel.x(), spVel.y(), spVel.z(), SPIN_ARGS_END);
        spin.NodeMessage(spin.getUserID().c_str(), "sfff", "setSpin", spSpin.x(), spSpin.y(), spSpin.z(), SPIN_ARGS_END);

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
    //osgViewer::Viewer::frame(f);
    osgViewer::CompositeViewer::frame(f);
    
    // initilize PPU if it was not done before
    //initializePPU();

    // compute frame time
    float frameTime = elapsedTime() - mOldTime;
    mOldTime = elapsedTime();

    // print also some info about the fps number
    if (0)
    {
        osgPPU::UnitText* ppu = dynamic_cast<osgPPU::UnitText*>(mProcessor->findUnit("FPSTextPPU"));
        if (ppu)
        {
            char txt[64];
            sprintf(txt, "FPS: %4.2f", 1.0 / frameTime);
            ppu->setText(txt);
        }
    }
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
    osg::Vec3Array* texcoords = new osg::Vec3Array;
    osg::Vec4Array* colors = new osg::Vec4Array;

    osg::Vec3 bottom = origin;
    osg::Vec3 dx = xAxis*(width/((float)(noSteps-1)));
    osg::Vec3 dy = yAxis*(height/((float)(noSteps-1)));
    
    osg::Vec3d screenCenter = origin + widthVector*0.5f + heightVector*0.5f;
    float screenRadius = heightVector.length() * 0.5f;

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
                texcoords->push_back(texcoord);

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
                texcoords->push_back(texcoord);

                cursor += dx;
            }
            // osg::notify(osg::NOTICE)<<std::endl;
        }
    }
    
    // pass the created vertex array to the points geometry object.
    geometry->setVertexArray(vertices);

    geometry->setColorArray(colors);
    geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

    geometry->setTexCoordArray(0,texcoords);

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
    texture->setFilter(osg::Texture::MIN_FILTER,osg::Texture::LINEAR);
    texture->setFilter(osg::Texture::MAG_FILTER,osg::Texture::LINEAR);
    texture->setWrap(osg::Texture::WRAP_S,osg::Texture::CLAMP_TO_EDGE);
    texture->setWrap(osg::Texture::WRAP_T,osg::Texture::CLAMP_TO_EDGE);
    texture->setWrap(osg::Texture::WRAP_R,osg::Texture::CLAMP_TO_EDGE);

#if 0
    osg::Camera::RenderTargetImplementation renderTargetImplementation = osg::Camera::SEPERATE_WINDOW;
    GLenum buffer = GL_FRONT;
#else
    osg::Camera::RenderTargetImplementation renderTargetImplementation = osg::Camera::FRAME_BUFFER_OBJECT;
    GLenum buffer = GL_FRONT;
#endif

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
        camera->attach(osg::Camera::COLOR_BUFFER, texture, 0, osg::TextureCubeMap::POSITIVE_Y);

        view->addSlave(camera.get(), osg::Matrixd(), osg::Matrixd());
    }

    // top face
    {
        osg::ref_ptr<osg::Camera> camera = new osg::Camera;
        camera->setName("Top face camera");
        camera->setGraphicsContext(gc);
        camera->setViewport(new osg::Viewport(0,0,camera_width, camera_height));
        GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
        camera->setDrawBuffer(buffer);
        camera->setReadBuffer(buffer);
        camera->setAllowEventFocus(false);

        // tell the camera to use OpenGL frame buffer object where supported.
        camera->setRenderTargetImplementation(renderTargetImplementation);

        // attach the texture and use it as the color buffer.
        camera->attach(osg::Camera::COLOR_BUFFER, texture, 0, osg::TextureCubeMap::POSITIVE_Z);

        view->addSlave(camera.get(), osg::Matrixd(), osg::Matrixd::rotate(osg::inDegrees(-90.0f), 1.0,0.0,0.0));
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
        camera->attach(osg::Camera::COLOR_BUFFER, texture, 0, osg::TextureCubeMap::NEGATIVE_X);

        view->addSlave(camera.get(), osg::Matrixd(), osg::Matrixd::rotate(osg::inDegrees(-90.0f), 0.0,1.0,0.0) * osg::Matrixd::rotate(osg::inDegrees(-90.0f), 0.0,0.0,1.0));
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
        camera->attach(osg::Camera::COLOR_BUFFER, texture, 0, osg::TextureCubeMap::POSITIVE_X);

        view->addSlave(camera.get(), osg::Matrixd(), osg::Matrixd::rotate(osg::inDegrees(90.0f), 0.0,1.0,0.0 ) * osg::Matrixd::rotate(osg::inDegrees(90.0f), 0.0,0.0,1.0));
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
        camera->attach(osg::Camera::COLOR_BUFFER, texture, 0, osg::TextureCubeMap::NEGATIVE_Z);

        view->addSlave(camera.get(), osg::Matrixd(), osg::Matrixd::rotate(osg::inDegrees(90.0f), 1.0,0.0,0.0) * osg::Matrixd::rotate(osg::inDegrees(180.0f), 0.0,0.0,1.0));
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
        camera->attach(osg::Camera::COLOR_BUFFER, texture, 0, osg::TextureCubeMap::NEGATIVE_Y);

        view->addSlave(camera.get(), osg::Matrixd(), osg::Matrixd::rotate(osg::inDegrees(180.0f), 1.0,0.0,0.0));
    }

    //view->getCamera()->setProjectionMatrixAsPerspective(90.0f, 1.0, 1, 1000.0);
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
        stateset->setTextureAttributeAndModes(0, texture,osg::StateAttribute::ON);
        stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

        if (!applyIntensityMapAsColours && intensityMap)
        {
            stateset->setTextureAttributeAndModes(1, new osg::Texture2D(intensityMap), osg::StateAttribute::ON);
        }

        // mikewoz: we pass the pre-made camera here:
        osg::ref_ptr<osg::Camera> camera = new osg::Camera;
        //osg::ref_ptr<osg::Camera> camera = cam;

        camera->setGraphicsContext(gc);
        camera->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT );
        camera->setClearColor( osg::Vec4(0.0,0.0,0.0,1.0) );
        camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
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

    //view->getCamera()->setNearFarRatio(0.0001f);
    cam->setNearFarRatio(0.0001f);
}



void loadXMLcamera(TiXmlElement *XMLnode, osgViewer::Viewer::View *view, osg::Camera *cam, osg::GraphicsContext::Traits *traits, osg::GraphicsContext *gc)
//static void loadXMLcamera(TiXmlElement *XMLnode, osgViewer::Viewer::View *view, osg::Camera *cam, int screenWidth, int screenHeight, int screenNum)
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
        std::cout << "    Loading camera: " << XMLnode->Attribute("id") << std::endl;
        cam->setName(XMLnode->Attribute("id"));
    }
    else std::cout << "    Loading camera" << std::endl;


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
        int textureSize = 2048;
        float radius = 1.0;
        float collar = 0.45;
        float distance = 0.0;
        float crop = 0.0;
        float near = 1.0;
        float far = 1000.0; 
        for ( child = XMLnode->FirstChildElement(); child; child = child->NextSiblingElement() )
	    {
		    if (child->FirstChild())
		    {
		    	tag = child->Value();
	    		val = child->FirstChild()->Value();
	    	} else continue;
        
		    if (tag=="textureSize")
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
        
        osg::Matrixd projMatrix = osg::Matrixd::identity();
        //osg::Matrixd projMatrix = osg::Matrixd::translate(osg::Vec3(0,0,1.0));
 
        std::cout << "creating spherical display with textureSize="<<textureSize<<", radius="<<radius<<", collar="<<collar<<", crop="<<crop<<std::endl;

        //view->setUpViewFor3DSphericalDisplay(radius, collar, screenNum, intensityMap, projMatrix);
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
	                //std::cout << "setting perspective of " << fovy << "deg, aspect: " << aspectRatio << std::endl;
	                cam->setProjectionMatrixAsPerspective(fovy, aspectRatio, zNear, zFar);
	            }
	        }
	    	else if (tag=="frustum")
		    {
                frustum frust;
                if (sscanf (val.c_str(),"%f %f %f %f %f %f",&frust.left,&frust.right,&frust.bottom,&frust.top,&frust.near,&frust.far))
                {
                    cam->setProjectionMatrixAsFrustum(frust.left, frust.right, frust.bottom, frust.top, frust.near, frust.far);
		        }   
            }

		    else
		    {
			    std::cout << "Unknown parameter in configuration file: " << tag << std::endl;
		    }
	    }

        //view->addSlave(cam, view->getCamera()->getProjectionMatrix(), view->getCamera()->getViewMatrix());
    }


	//cam->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);

    //cam->setViewMatrixAsLookAt(eye, lookat, up);

    ViewerManipulator *manipulator = new spin::ViewerManipulator();
    manipulator->setHomePosition( eye, lookat, up, false );
    view->setCameraManipulator(manipulator);


/*
	// note: the first matrix scales and offsets the axes (perspective) while the second matrix offsets the view:
	//viewer.addSlave(cam->camera.get(), cam->pMatrix*cam->tMatrix, cam->rMatrix);
	//cam->camera->setViewMatrixAsLookAt( cam->_eye, cam->_lookat, cam->_up );
	osg::Matrixd viewMatrix;
	viewMatrix.makeLookAt( cam->_eye, cam->_lookat, cam->_up );
	viewMatrix *= osg::Matrixd::rotate(osg::PI/2, X_AXIS);
	viewer.addSlave(cam->camera.get(), cam->pMatrix*cam->tMatrix, cam->rMatrix*viewMatrix);
*/

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
	//osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits(view->getDisplaySettings());
	//osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits(ds);
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits();

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
        if (boost::iequals(n->FirstChild()->Value(), "false"))
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
        if (boost::iequals(n->FirstChild()->Value(), "false"))
            traits->useCursor = false;
        else
            traits->useCursor = true;
    }

    if ((n = XMLnode->FirstChildElement("windowDecoration")))
    {
        if (boost::iequals(n->FirstChild()->Value(), "false"))
            traits->windowDecoration = false;
        else
            traits->windowDecoration = true;
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
    
    if ((theMethod=="setParam") && (stringArgs.size()==1) && (floatArgs.size()==1))
    {
    
        if (stringArgs[0] == "gaussSigma")
        {
            viewer->dofPPU_->setGaussSigma(floatArgs[0]);
        }
        else if (stringArgs[0] == "gaussRadius")
        {
            viewer->dofPPU_->setGaussRadius(floatArgs[0]);
        }
        else if (stringArgs[0] == "focalLength")
        {
            viewer->dofPPU_->setFocalLength(floatArgs[0]);
        }
        else if (stringArgs[0] == "focalRange")
        {
            viewer->dofPPU_->setFocalRange(floatArgs[0]);
        }
        else if (stringArgs[0] == "near")
        {
            viewer->dofPPU_->setNear(floatArgs[0]);
        }
        else if (stringArgs[0] == "far")
        {
            viewer->dofPPU_->setFar(floatArgs[0]);
        }
        
        return 0;
    }
    else if ((theMethod=="setFrustum") && (floatArgs.size()==6))
    {
        viewer->getView(0)->getCamera()->setProjectionMatrixAsFrustum(floatArgs[0], floatArgs[1], floatArgs[2], floatArgs[3], floatArgs[4], floatArgs[5]);
        
        viewer->dofPPU_->setNear(floatArgs[4]);
        viewer->dofPPU_->setFar(floatArgs[5]);
        
        return 0;
    }
    else if ((theMethod=="setVelocityScalars") && (floatArgs.size()==3))
    {
	viewer->setVelocityScalars(osg::Vec3(floatArgs[0], floatArgs[1], floatArgs[2]));
	return 0;
    }
    else if ((theMethod=="setSpinScalars") && (floatArgs.size()==3))
    {
	viewer->setSpinScalars(osg::Vec3(floatArgs[0], floatArgs[1], floatArgs[2]));
	return 0;
    }
    
    return 1;
}



} // end namespace
