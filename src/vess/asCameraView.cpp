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

#include <iostream>

#include <osg/GraphicsContext>
//#include <osgViewer/Renderer>
#include <osgViewer/View>
//#include <osgViewer/GraphicsWindow>


#include <osg/GLExtensions>

#include <osgGA/NodeTrackerManipulator>
#include <osgGA/TrackballManipulator>
#include <osgViewer/ViewerEventHandlers>
#include <osgViewer/CompositeViewer>
#include <osg/TextureCubeMap>
#include <osg/Image>



#include "asUtil.h"
#include "asCameraView.h"

using namespace std;


asCameraView::asCameraView(const char *id, osgViewer::CompositeViewer &viewer, osg::ref_ptr<asWindow> win)
{	
	this->id = id;
	this->win = win;
	

	distortionType = "NONE";
	distortionGeode = new osg::Geode(); // keep a geode available

	// First, create the view and assign a gfxContext:
	view = new osgViewer::View;
	
	if (win.valid())
	{
		view->getCamera()->setGraphicsContext(win->getGfxContext());
	} else {
		std::cout << "ERROR (asCameraView): No valid window provided!" << std::endl;
	}
	
	view->getCamera()->setName(std::string(id) + ".asCameraView");
	//view->getCamera()->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT );


	// *************************************************************************
	// set up the Manipulator:

	
	//osgGA::TrackballManipulator *manipulator = new osgGA::TrackballManipulator();
	osgGA::NodeTrackerManipulator *manipulator = new osgGA::NodeTrackerManipulator();

	//manipulator->setTrackerMode( osgGA::NodeTrackerManipulator::NODE_CENTER );
	manipulator->setTrackerMode( osgGA::NodeTrackerManipulator::NODE_CENTER_AND_ROTATION );
	//manipulator->setTrackerMode( osgGA::NodeTrackerManipulator::NODE_CENTER_AND_AZIM );

	manipulator->setRotationMode( osgGA::NodeTrackerManipulator::ELEVATION_AZIM );
	//manipulator->setRotationMode( osgGA::NodeTrackerManipulator::TRACKBALL );

	manipulator->setMinimumDistance ( 0.0001 );
	manipulator->setHomePosition( osg::Vec3(0,-1,0), osg::Vec3(0,0,0), osg::Vec3(0,0,1), false );
	
	manipulator->setTrackNode(this);
	view->setCameraManipulator(manipulator);
	

    
	
	// *************************************************************************
	// set up some event (GUIEventHandler) handlers:
	
	view->addEventHandler(new osgViewer::StatsHandler);
	view->addEventHandler(new osgViewer::ThreadingHandler);
	view->addEventHandler(new osgViewer::WindowSizeHandler);
	
	//view->addEventHandler(new osgViewer::KeyboardHandler);
	//view->addEventHandler(new osgViewer::MouseHandler);
	//view->addEventHandler(new osgViewer::ResizeHandler);
	
	view->addEventHandler(new osgViewer::RecordCameraPathHandler);
	
	
	
	//view->setLightingMode(osg::View::NO_LIGHT);
	view->setLightingMode(osg::View::HEADLIGHT);
	//view->setLightingMode(osg::View::SKY_LIGHT);
	 
	/*
	GLenum buffer = win->getGfxTraits()->doubleBuffer ? GL_BACK : GL_FRONT;
	view->getCamera()->setDrawBuffer(buffer);
	view->getCamera()->setReadBuffer(buffer);
	*/
	
	setTranslation(0,0,0);
	setOrientation(0,0,0);
	setViewport(0,0,1,1);
	
	// we set the update flag... so next time the asCameraManager is updated
	// it will add the camera to the viewer
	additionFlag = true;
	removalFlag = false;
	distortionUpdate = false;
}

asCameraView::~asCameraView()
{
	// should do this:
	//viewer->removeView();
	
}

// *****************************************************************************


void asCameraView::setFrustum (float left, float right, float bottom, float top, float zNear, float zFar)
{
	view->getCamera()->setProjectionMatrixAsFrustum( left, right, bottom, top, zNear, zFar );
}

void asCameraView::setViewport (float x, float y, float w, float h)
{
	osg::GraphicsContext::Traits *traits = win->getGfxTraits();
	
	_viewport = osg::Vec4(x,y,w,h);

	//view->getCamera()->setViewport( (int) x, (int) y, (int) w, (int) h );
	
	// TODO: iterate over all slave cameras in view:
	view->getCamera()->setViewport( (int) (x*traits->width), (int) (y*traits->height), (int) (w*traits->width), (int) (h*traits->height) );
}

void asCameraView::updateViewport()
{
	setViewport(_viewport.x(), _viewport.y(), _viewport.z(), _viewport.w());
}


void asCameraView::setClearColor (float r, float g, float b, float a)
{
	view->getCamera()->setClearColor(osg::Vec4f(r,g,b,a));
}


void asCameraView::setTranslation (float x, float y, float z)
{
	this->setPosition(osg::Vec3d(x,y,z));
}


void asCameraView::setOrientation (float p, float r, float y)
{
	_orientation = osg::Vec3(p,r,y);
	
	// note: args are passed in degrees; must convert to radians:
	osg::Quat q = osg::Quat( osg::DegreesToRadians(p), osg::Vec3d(1,0,0),
							 osg::DegreesToRadians(r), osg::Vec3d(0,1,0),
							 osg::DegreesToRadians(y), osg::Vec3d(0,0,1));
	this->setAttitude(q);


	//osg::Matrix m;
	//view->getCamera()->

}

void asCameraView::move (float x, float y, float z)
{
	osg::Vec3 newPos = this->getPosition() + ( this->getAttitude() * osg::Vec3(x,y,z) );
	this->setPosition(newPos);
}

void asCameraView::rotate (float p, float r, float y)
{
	this->setOrientation(_orientation.x()+p, _orientation.y()+r, _orientation.z()+y);
}


void asCameraView::setDistortion(string type, vector<float> args)
{
	
	if (1) // (type != this->distortionType)
	{
		// User has changed distortion types. We set addition flag so that the 
		// new distortion with all related cameras will be created in the next
		// asCameraManager::update() call (once all traversals are complete).
		
		this->distortionType = type;
		this->distortionArgs = args;
		this->distortionUpdate = true;	
	}
	
	/*
	// If we received this message, then we can assume that the args have been
	// changed and that a new distortionGeometry needs to be generated. Thus,
	// the first thing to do is to remove the existing geometry:
	

	if (distortionGeode->containsDrawable(distortionGeometry.get()))
	{
		distortionGeode->removeDrawable(distortionGeometry.get());
		distortionGeometry = NULL;
	}
	
	
	
	// now create a new geometry based on the distortionType:
	if (distortionType=="NONE")
	{
		// nothing to do here
	}

	else if ((distortionType=="DOME") && (args.size()==2))
	{
		osg::GraphicsContext::Traits *traits = win->getGfxTraits();
		
	    unsigned int width = traits->width;
	    unsigned int height = traits->height;
	    
	    osg::Image *intensityMap= new osg::Image();
		osg::Matrixd *projectorMatrix = new osg::Matrixd;
		
		intensityMap=0;
		
		float radius = args[0];
		float collar = args[1];
		
		bool applyIntensityMapAsColours = true;
		

				
		// build new geometry:
		distortionGeometry = create3DSphericalDisplayDistortionMesh(osg::Vec3(0.0f,0.0f,0.0f), osg::Vec3(width,0.0f,0.0f), osg::Vec3(0.0f,height,0.0f), radius, collar, applyIntensityMapAsColours ? intensityMap : 0, *projectorMatrix);
		distortionGeode->addDrawable(distortionGeometry.get());
		
    	// new we need to add the texture to the mesh, we do so by creating a 
    	// StateSet to contain the Texture StateAttribute.
    	osg::StateSet* stateset = distortionGeode->getOrCreateStateSet();
    	stateset->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);
    	stateset->setMode(GL_LIGHTING,osg::StateAttribute::ON);

    	if (!applyIntensityMapAsColours && intensityMap)
    	{
    		stateset->setTextureAttributeAndModes(1, new osg::Texture2D(intensityMap), osg::StateAttribute::ON);
    	}
	    
	}
	*/
    
}


// *****************************************************************************


void asCameraView::makePlanar()
{
	
	if (distortionGeode->containsDrawable(distortionGeometry.get()))
	{
		distortionGeode->removeDrawable(distortionGeometry.get());
		distortionGeometry = NULL;
	}
	
	while (view->getNumSlaves())
	{
		view->removeSlave(0);
	}
	
	//reset the main camera:
	//view->getCamera()
	
}

void asCameraView::makeDome()
{
	// we have to clear any existing distortion before we make new stuff,
	// so we just call makePlanar() first:
	this->makePlanar();
	
	float radius, collar;
	
	if (distortionArgs.size()==2)
	{		
		radius = this->distortionArgs[0];
		collar = this->distortionArgs[1];
	}
	else
	{
		std::cout << "ERROR: makeDome() called with " << distortionArgs.size() << " args" << std::endl;
		return;
	}
	
	
	osg::ref_ptr<osg::GraphicsContext> gc = win->getGfxContext();
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = win->getGfxTraits();
	
    unsigned int width = traits->width;
    unsigned int height = traits->height;

	
	
	// FROM osg::View::setUpViewFor3DSphericalDisplay()
	/*
    osg::GraphicsContext::ScreenIdentifier si;
    si.readDISPLAY();
    
    // displayNum has not been set so reset it to 0.
    if (si.displayNum<0) si.displayNum = 0;

    si.screenNum = 0;

    osg::GraphicsContext::WindowingSystemInterface* wsi = osg::GraphicsContext::getWindowingSystemInterface();
    wsi->getScreenResolution(si, width, height);


    traits = new osg::GraphicsContext::Traits;
    traits->hostName = si.hostName;
    traits->displayNum = si.displayNum;
    traits->screenNum = si.screenNum;
    traits->x = 0;
    traits->y = 0;
    traits->width = width;
    traits->height = height;
    traits->windowDecoration = false;
    traits->doubleBuffer = true;
    //traits->sharedContext = win->getGfxContext();
    traits->sharedContext = 0;
 
    gc = osg::GraphicsContext::createGraphicsContext(traits.get());
	*/

	
	

    osg::Image *intensityMap= new osg::Image();
	osg::Matrixd *projectorMatrix = new osg::Matrixd;
	
	intensityMap=0;
	
	bool applyIntensityMapAsColours = false;
    
    int tex_width = 512;
    int tex_height = 512;

    int camera_width = tex_width;
    int camera_height = tex_height;
    
    
    
	std::cout << "makeDome with radius="<<radius<<", collar="<<collar<<", width="<<width<<", height="<<height<<""<<std::endl;

    
    osg::TextureCubeMap* texture = new osg::TextureCubeMap;

    texture->setTextureSize(tex_width, tex_height);
    texture->setInternalFormat(GL_RGB);
    texture->setFilter(osg::Texture::MIN_FILTER,osg::Texture::LINEAR);
    texture->setFilter(osg::Texture::MAG_FILTER,osg::Texture::LINEAR);
    //texture->setWrap(osg::Texture::WRAP_S,osg::Texture::CLAMP_TO_EDGE);
    //texture->setWrap(osg::Texture::WRAP_T,osg::Texture::CLAMP_TO_EDGE);
    //texture->setWrap(osg::Texture::WRAP_R,osg::Texture::CLAMP_TO_EDGE);

    //osg::Camera::RenderTargetImplementation renderTargetImplementation = osg::Camera::SEPERATE_WINDOW;
    //osg::Camera::RenderTargetImplementation renderTargetImplementation = osg::Camera::FRAME_BUFFER;
    osg::Camera::RenderTargetImplementation renderTargetImplementation = osg::Camera::FRAME_BUFFER_OBJECT;
    //osg::Camera::RenderTargetImplementation renderTargetImplementation = osg::Camera::PIXEL_BUFFER_RTT;
    GLenum buffer = GL_FRONT;
    
    
    // front face
    {
        osg::ref_ptr<osg::Camera> camera = new osg::Camera;
        camera->setName("Front face camera");
        camera->setGraphicsContext(gc.get());
        camera->setViewport(new osg::Viewport(0,0,camera_width, camera_height));
        camera->setDrawBuffer(buffer);
        camera->setReadBuffer(buffer);
        camera->setAllowEventFocus(false);
        // tell the camera to use OpenGL frame buffer object where supported.
        camera->setRenderTargetImplementation(renderTargetImplementation);

        // attach the texture and use it as the color buffer.
        camera->attach(osg::Camera::COLOR_BUFFER, texture, 0, osg::TextureCubeMap::POSITIVE_Y);

        camera->setRenderOrder(osg::Camera::PRE_RENDER);
        
        view->addSlave(camera.get(), osg::Matrixd(), osg::Matrixd());
    }


    // top face
    {
        osg::ref_ptr<osg::Camera> camera = new osg::Camera;
        camera->setName("Top face camera");
        camera->setGraphicsContext(gc.get());
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
        camera->setGraphicsContext(gc.get());
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
        camera->setGraphicsContext(gc.get());
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
        camera->setGraphicsContext(gc.get());
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
        camera->setGraphicsContext(gc.get());
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

    view->getCamera()->setProjectionMatrixAsPerspective(90.0f, 1.0, 1, 1000.0);


    // distortion correction set up.
    {
		// build new distortionGeometry:
		//distortionGeometry = create3DSphericalDisplayDistortionMesh(osg::Vec3(0.0f,0.0f,0.0f), osg::Vec3(width,0.0f,0.0f), osg::Vec3(0.0f,height,0.0f), radius, collar, applyIntensityMapAsColours ? intensityMap : 0, *projectorMatrix);
		distortionGeometry = createDomeDistortionMesh(osg::Vec3(0.0f,0.0f,0.0f), osg::Vec3(width,0.0f,0.0f), osg::Vec3(0.0f,height,0.0f), radius, collar);
		
		distortionGeode->addDrawable(distortionGeometry.get());

    	// new we need to add the texture to the mesh, we do so by creating a 
    	// StateSet to contain the Texture StateAttribute.
    	osg::StateSet* stateset = distortionGeode->getOrCreateStateSet();
    	stateset->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);
    	stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

    	if (!applyIntensityMapAsColours && intensityMap)
    	{
    		std::cout << "Got here (intensity map?)" << std::endl;
    		stateset->setTextureAttributeAndModes(1, new osg::Texture2D(intensityMap), osg::StateAttribute::ON);
    	}
		
    	osg::ref_ptr<osg::Camera> camera = new osg::Camera;
    	camera->setGraphicsContext(gc.get());
    	camera->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT );
    	camera->setClearColor( osg::Vec4(1.0,0.7,0.8,0.0) );
    	//camera->setViewport(new osg::Viewport(0, 0, width, height));
    	//camera->setViewport( new osg::Viewport( (int) (_viewport.x()*width), (int) (_viewport.y()*height), (int) (_viewport.z()*width), (int) (_viewport.w()*height) ));
    	camera->setViewport( view->getCamera()->getViewport() );
    	
    	GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
    	camera->setDrawBuffer(buffer);
    	camera->setReadBuffer(buffer);
    	camera->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
    	camera->setAllowEventFocus(false);
    	//camera->setAllowEventFocus(true);
    	camera->setInheritanceMask(camera->getInheritanceMask() & ~osg::CullSettings::CLEAR_COLOR & ~osg::CullSettings::COMPUTE_NEAR_FAR_MODE);
    	//camera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
    	
    	camera->setProjectionMatrixAsOrtho2D(0,width,0,height);
    	camera->setViewMatrix(osg::Matrix::identity());

    	// add subgraph to render
    	camera->addChild(distortionGeode.get());
    	

    	camera->setName("DistortionCorrectionCamera");

    	view->addSlave(camera.get(), osg::Matrixd(), osg::Matrixd(), false);
    

    	
    	
    	//camera->setRenderOrder(osg::Camera::POST_RENDER);
    	//view->getCamera()->setDrawBuffer(GL_BACK);
    	//view->getCamera()->setReadBuffer(GL_BACK);
    
    }

    view->getCamera()->setNearFarRatio(0.0001f);
    
    /*
    if (view->getLightingMode()==osg::View::HEADLIGHT)
    {
        // set a local light source for headlight to ensure that lighting is consistent across sides of cube.
    	view->getLight()->setPosition(osg::Vec4(0.0f,0.0f,0.0f,1.0f));
    }
    */
	
}



static osg::Geometry* create3DSphericalDisplayDistortionMesh(const osg::Vec3& origin, const osg::Vec3& widthVector, const osg::Vec3& heightVector, double sphere_radius, double collar_radius,osg::Image* intensityMap, const osg::Matrix& projectorMatrix)
{
    osg::Vec3d center(0.0,0.0,0.0);
    osg::Vec3d eye(0.0,0.0,0.0);

    double distance = 0;
    distance = sqrt(sphere_radius*sphere_radius - collar_radius*collar_radius);

    bool centerProjection = false;

    osg::Vec3d projector = eye - osg::Vec3d(0.0,0.0, distance);

    //osg::notify(osg::INFO)<<"create3DSphericalDisplayDistortionMesh : Projector position = "<<projector<<std::endl;
    //osg::notify(osg::INFO)<<"create3DSphericalDisplayDistortionMesh : distance = "<<distance<<std::endl;


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
    osg::Vec2Array* texcoords1 = intensityMap==0 ? new osg::Vec2Array : 0;
    osg::Vec4Array* colors = new osg::Vec4Array;

    osg::Vec3 bottom = origin;
    osg::Vec3 dx = xAxis*(width/((float)(noSteps-1)));
    osg::Vec3 dy = yAxis*(height/((float)(noSteps-1)));

    osg::Vec3d screenCenter = origin + widthVector*0.5f + heightVector*0.5f;
    float screenRadius = heightVector.length() * 0.5f;

    osg::Vec3 cursor = bottom;
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

                if (theta<0.0) theta += 2.0*osg::PI;

                // osg::notify(osg::NOTICE)<<"theta = "<<theta<< "phi="<<phi<<std::endl;

                osg::Vec3 texcoord(sin(phi) * cos(theta),
                                   sin(phi) * sin(theta),
                                   cos(phi));

                vertices->push_back(cursor);
                texcoords0->push_back(texcoord * projectorMatrix);

                osg::Vec2 texcoord1(theta/(2.0*osg::PI), 1.0f - phi/osg::PI_2);
                if (intensityMap)
                {
                	// TODO
                    //colors->push_back(intensityMap->getColor(texcoord1));
                }
                else
                {
                    colors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
                    if (texcoords1) texcoords1->push_back( texcoord1 );
                }

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
                if (theta<0.0) theta += 2.0*osg::PI;

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
                texcoords0->push_back(texcoord * projectorMatrix);

                osg::Vec2 texcoord1(theta/(2.0*osg::PI), 1.0f - phi/osg::PI_2);
                if (intensityMap)
                {
                	// TODO
                    //colors->push_back(intensityMap->getColor(texcoord1));
                }
                else
                {
                    colors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
                    if (texcoords1) texcoords1->push_back( texcoord1 );
                }

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
    if (texcoords1) geometry->setTexCoordArray(1,texcoords1);

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

static osg::Geometry* createDomeDistortionMesh(const osg::Vec3& origin, const osg::Vec3& widthVector, const osg::Vec3& heightVector,
                                        double sphere_radius, double collar_radius)
{

    osg::Vec3d center(0.0,0.0,0.0);
    osg::Vec3d eye(0.0,0.0,0.0);
    
    double distance = sqrt(sphere_radius*sphere_radius - collar_radius*collar_radius);
    //if (arguments.read("--distance", distance)) {}
    
    bool centerProjection = false;

    osg::Vec3d projector = eye - osg::Vec3d(0.0,0.0, distance);
    
    
    //osg::notify(osg::NOTICE)<<"Projector position = "<<projector<<std::endl;
    //osg::notify(osg::NOTICE)<<"distance = "<<distance<<std::endl;


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

    osg::Vec3 cursor = bottom;
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

