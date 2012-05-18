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

#include <osgViewer/CompositeViewer>
#include <osgViewer/View>
#include <osgViewer/ViewerEventHandlers>
#include <osg/GraphicsContext>
#include <osg/TextureCubeMap>

#include <osgGA/NodeTrackerManipulator>
#include <osgGA/TrackballManipulator>
#include <osgGA/StateSetManipulator>

#include <osgDB/ReadFile>
#include <osg/Timer>

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


extern pthread_mutex_t sceneMutex;

struct frustum
{
	bool valid;
	float left;
	float right;
	float bottom;
	float top;
	float near;
	float far;
};

static osg::Geometry* create3DSphericalDisplayDistortionMesh(const osg::Vec3& origin, const osg::Vec3& widthVector, const osg::Vec3& heightVector, double sphere_radius, double collar_radius, double distance=0)
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


static void makeDomeView(osg::GraphicsContext *gc, osg::GraphicsContext::Traits *traits, osg::View *view, osg::Camera *cam, int textureSize, double radius, double collar, double distance, double crop, osg::Image* intensityMap, const osg::Matrixd& projectorMatrix)
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



static void loadXMLcamera(TiXmlElement *XMLnode, osgViewer::Viewer::View *view, osg::Camera *cam, osg::GraphicsContext::Traits *traits, osg::GraphicsContext *gc)
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

    spin::ViewerManipulator *manipulator = new spin::ViewerManipulator();
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




static void loadXMLwindow(TiXmlElement *XMLnode, osgViewer::CompositeViewer &viewer)
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
    view->setSceneData(spin::spinApp::Instance().sceneManager->rootNode.get());
    viewer.addView(view.get());

    osgViewer::GraphicsWindow* gw = dynamic_cast<osgViewer::GraphicsWindow*>(gc.get());
    if (gw)
    {
        gw->getEventQueue()->getCurrentEventState()->setWindowRectangle(0, 0, traits->width, traits->height );
    }

}





int run(int argc, char **argv)
{
	//std::cout <<"\nspinViewer launching..." << std::endl;
    using namespace spin;
	spinClientContext spinListener;
	spinApp &spin = spinApp::Instance();

	std::string userID;
	bool picker = false;
	bool mover = true;
	
    int multisamples = 4;
	bool fullscreen = false;
	bool hideCursor=false;

	double maxFrameRate = 60;
	
	int x=50;
	int y=50;
	int width=640;
	int height=480;
	int screen=-1;

    double nearClipping = -1;
    double farClipping = -1;
	
	std::string camConfig;
	std::string sceneID = spin.getSceneID();
	
    //osg::setNotifyLevel(osg::INFO);
    
    // *************************************************************************

	// get arguments:
	osg::ArgumentParser arguments(&argc,argv);
	
	// set up the usage document, which a user can acess with -h or --help
    arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
	arguments.getApplicationUsage()->setDescription(arguments.getApplicationName()+" is a 3D viewer for the SPIN Framework.");
	arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" [options]");
	
    // add generic spin arguments (for address/port setup, scene-id, etc)
    spinListener.addCommandLineOptions(&arguments);

    // arguments specific to spinviewer:	
    arguments.getApplicationUsage()->addCommandLineOption("--user-id <uniqueID>", "Specify a user ID for this viewer (Default: <this computer's name>)"); 
	arguments.getApplicationUsage()->addCommandLineOption("--fullscreen", "Expand viewer to fullscreen");
    arguments.getApplicationUsage()->addCommandLineOption("--hide-cursor", "Hide the mouse cursor");
	arguments.getApplicationUsage()->addCommandLineOption("--config <filename>", "Provide a configuration file to customize the setup of multiple windows/cameras.");
	arguments.getApplicationUsage()->addCommandLineOption("--window <x y w h>", "Set the position (x,y) and size (w,h) of the viewer window (Default: 50 50 640 480)");
	arguments.getApplicationUsage()->addCommandLineOption("--clipping <near far>", "Manually specify fixed clipping planes (Default: clipping planes will be recomputed for every frame)");
    arguments.getApplicationUsage()->addCommandLineOption("--screen <num>", "Screen number to display on (Default: ALLSCREENS)");
	arguments.getApplicationUsage()->addCommandLineOption("--framerate <num>", "Set the maximum framerate (Default: not limited)");
	arguments.getApplicationUsage()->addCommandLineOption("--multisamples <num>", "Set the level of multisampling for antialiasing (Default: 4)");
	arguments.getApplicationUsage()->addCommandLineOption("--disable-camera-controls", "Disable mouse-baed camera controls for this user. This is helpful when using a mouse picker.");
	arguments.getApplicationUsage()->addCommandLineOption("--enable-mouse-picker", "Enable the mouse picker, and send events to the server");

	// *************************************************************************
	// PARSE ARGS:

    if (!spinListener.parseCommandLineOptions(&arguments))
        return 0;
    
	osg::ArgumentParser::Parameter param_userID(userID);
	arguments.read("--user-id", param_userID);
    if (not userID.empty())
        spin.setUserID(userID);

	osg::ArgumentParser::Parameter param_camConfig(camConfig);
	arguments.read("--config", param_camConfig);
   
	frustum frust;
	frust.valid = false;
	while (arguments.read("--frustum", frust.left, frust.right, frust.bottom, frust.top)) {
		frust.near = 1.0;
		frust.far = 10000.0;
		frust.valid = true;
	}
	while (arguments.read("--clipping",nearClipping,farClipping)) {}
    if (arguments.read("--fullscreen")) fullscreen=true;
    if (arguments.read("--hide-cursor")) hideCursor=true;
	while (arguments.read("--window",x,y,width,height)) {}
	while (arguments.read("--screen",screen)) {}
	while (arguments.read("--framerate",maxFrameRate)) {}
	while (arguments.read("--multisamples",multisamples)) {}
	if (arguments.read("--disable-camera-controls")) mover=false;
	if (arguments.read("--enable-mouse-picker")) picker=true;


	// For testing purposes, we allow loading a scene with a commandline arg:
    //std::cout << "DYLD_LIBRARY_PATH= " << getenv("DYLD_LIBRARY_PATH") << std::endl;
    //std::cout << "OSG_LIBRARY_PATH=  " << getenv("OSG_LIBRARY_PATH") << std::endl;
	//setenv("OSG_PLUGIN_EXTENSION", ".so", 1);
    osg::ref_ptr<osg::Node> argScene = osgDB::readNodeFiles(arguments);
    
    
	// *************************************************************************
	// construct the viewer:
	// (note, this constructor gets rid of some additional args)

	osgViewer::CompositeViewer viewer = osgViewer::CompositeViewer(arguments);
    //viewer.setThreadingModel(osgViewer::CompositeViewer::AutomaticSelection);
	viewer.setThreadingModel(osgViewer::CompositeViewer::SingleThreaded);
	//viewer.setThreadingModel(osgViewer::CompositeViewer::CullDrawThreadPerContext);

	viewer.getUsage(*arguments.getApplicationUsage());

    osg::DisplaySettings::instance()->setNumMultiSamples( multisamples );

	// *************************************************************************
	// start the listener thread:

	if (!spinListener.start())
	{
        std::cout << "ERROR: could not start SPIN listener" << std::endl;
        exit(EXIT_FAILURE);
	}

	spin.sceneManager->setGraphical(true);

	// *************************************************************************
	// get details on keyboard and mouse bindings used by the viewer.
	viewer.getUsage(*arguments.getApplicationUsage());

    // *************************************************************************
    // set up initial view:

    osg::ref_ptr<ViewerManipulator> manipulator;


	// ***************************************************************************

	if (camConfig.size())
	{
		std::cout << "Loading config file (" << camConfig << ")..." << std::endl;
		
		TiXmlDocument doc( camConfig.c_str() );

		TiXmlNode *root = 0;
		TiXmlElement *child = 0;

		// Load the XML file and verify:
		if ( !doc.LoadFile() ) {
			std::cout << "WARNING: failed to load " << camConfig << ". Invalid XML format." << std::endl;
            return 1;
        }

		// get the <camConfig> tag and verify:
		if (!(root = doc.FirstChild( "camConfig" )))
		{
			std::cout << "WARNING: failed to load " << camConfig << ". XML file has no <camConfig> tag." << std::endl;
			return false;
            return 1;
		}
		
		// okay.. we have a valid xml file.

		// look for cameras:
		for ( child = root->FirstChildElement("window"); child; child = child->NextSiblingElement("window") )
		{
			loadXMLwindow(child, viewer);
		}
	
	}

    else
    {
	    osg::ref_ptr<osgViewer::View> view = new osgViewer::View;
        viewer.addView(view.get());

        view->getCamera()->setClearColor(osg::Vec4(0.0, 0.0, 0.0, 0.0));

        if (fullscreen)
        {
        	if (screen<0) view->setUpViewAcrossAllScreens();
        	else view->setUpViewOnSingleScreen(screen);
        } else {
        	if (screen<0) view->setUpViewInWindow(x,y,width,height);
        	else view->setUpViewInWindow(x,y,width,height,screen);
        }

        if (frust.valid)
        {
        	//view->getCamera()->getProjectionMatrixAsFrustum(frust.left, frust.right, frust.bottom, frust.top, frust.near, frust.far);
        	std::cout << "  Custom frustum:\t\t" << frust.left<<" "<<frust.right<<" "<<frust.bottom<<" "<<frust.top<<" "<<frust.near<<" "<<frust.far << std::endl;
        	view->getCamera()->setProjectionMatrixAsFrustum(frust.left, frust.right, frust.bottom, frust.top, frust.near, frust.far);
        }

        //TODO:2010-07-28:aalex:Load an image for a window icon
        //HANDLE icon = osg::LoadImage(0, "MyIcon.ico", osg::IMAGE_ICON, 0, 0, osg::LR_LOADFROMFILE);
        osgViewer::ViewerBase::Windows windows;
        osgViewer::ViewerBase::Windows::iterator wIter;
        viewer.getWindows(windows);
        for (wIter=windows.begin(); wIter!=windows.end(); wIter++)
        {
        	(*wIter)->setWindowName("spinviewer " + spin.getUserID() + "@" + spin.getSceneID());
		    if (hideCursor) (*wIter)->useCursor(false);

            //TODO:2010-07-28:aalex:Set a window icon
            //if( hIcon && hWnd )
            //{
            //    osg::SendMessage(hWnd, WM_SETICON, ICON_SMALL, (LPARAM)hIcon);
            //} 
        }

        view->setSceneData(spin.sceneManager->rootNode.get());

	    view->addEventHandler(new osgViewer::StatsHandler);
	    view->addEventHandler(new osgViewer::ThreadingHandler);
	    view->addEventHandler(new osgViewer::WindowSizeHandler);

	    view->addEventHandler(new osgViewer::HelpHandler(arguments.getApplicationUsage()));
	    view->addEventHandler( new osgGA::StateSetManipulator(view->getCamera()->getOrCreateStateSet()) );

	    //view->setLightingMode(osg::View::NO_LIGHT);
	    view->setLightingMode(osg::View::HEADLIGHT);
	    //view->setLightingMode(osg::View::SKY_LIGHT);

        view->setCameraManipulator(new ViewerManipulator());
    }

    // *************************************************************************
    // disable OSG's auto computation of near/far clipping planes, and replace
    // the with our own:
    if ((nearClipping>=0) && (farClipping>nearClipping))
    {
        osgViewer::Viewer::Cameras cameras;
        viewer.getCameras(cameras);
        for (osgViewer::Viewer::Cameras::iterator iter = cameras.begin(); iter != cameras.end(); ++iter)
        {
            (*iter)->setNearFarRatio(0.0001f);
            double fovy, aspectRatio, zNear, zFar;
            (*iter)->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
            (*iter)->getProjectionMatrixAsPerspective(fovy, aspectRatio, zNear, zFar);
            (*iter)->setProjectionMatrixAsPerspective(fovy, aspectRatio, nearClipping, farClipping);
            
            // for point clouds, let's disable small feature culling:
            /*
            (*iter)->setCullingMode((*iter)->getCullingMode() & ~osg::CullSettings::SMALL_FEATURE_CULLING);
            osg::View *v = (*iter)->getView();
            for (unsigned int slaveNum=0; slaveNum<v->getNumSlaves(); slaveNum++)
            {
                osg::Camera *slaveCam = v->getSlave(slaveNum)._camera.get();
                slaveCam->setCullingMode(slaveCam->getCullingMode() & ~osg::CullSettings::SMALL_FEATURE_CULLING);
            }
            */
        }
    }


    // *************************************************************************
    // create a camera manipulator
/*
    manipulator = new ViewerManipulator();
    manipulator->setPicker(picker);
    manipulator->setMover(mover);
*/
    
    for (unsigned int i=0; i<viewer.getNumViews(); i++)
    {
        ViewerManipulator *vm = dynamic_cast<ViewerManipulator*>(viewer.getView(i)->getCameraManipulator());
        if (vm)
        {
            vm->setPicker(picker);
            vm->setMover(mover);
        }
        //viewer.getView(i)->setCameraManipulator(manipulator.get());
    }
        
	// ***************************************************************************
	// debug print camera info
	
	if (0)
	{
        std::cout << std::endl << "CAMERA DEBUG PRINT:" << std::endl;
        
		osgViewer::Viewer::Cameras cameras;
		viewer.getCameras(cameras);
		
		osg::Vec3d eye, center, up;
		double left, right, bottom, top, zNear, zFar;

		if (manipulator.valid())
        {
            manipulator->getHomePosition (eye, center, up);
		    std::cout << "   manipulator:    eye=(" << eye.x() << "," << eye.y() << "," << eye.z() << ") center=(" << center.x() << "," << center.y() << "," << center.z() << ") up=(" << up.x() << "," << up.y() << "," << up.z() << ")" << std::endl;
        }
		for (osgViewer::Viewer::Cameras::iterator iter = cameras.begin(); iter != cameras.end(); ++iter)
		{
            std::cout << "Camera '" << (*iter)->getName() << "':" << std::endl;
            
			osg::Vec4 v4 = (*iter)->getClearColor();
			std::cout << "   clear color:      (" << v4.x() << "," << v4.y() << "," << v4.z() << "," << v4.w() << ")" << std::endl;
			
			(*iter)->getProjectionMatrixAsFrustum (left, right, bottom, top, zNear, zFar);
			std::cout << "   Frustum:          " << left << "," << right << "," << top << "," << bottom << "  clip: " << zNear << "-" << zFar << std::endl;

			(*iter)->getViewMatrixAsLookAt (eye, center, up);
			std::cout << "   Camera LookAt:    eye=(" << eye.x() << "," << eye.y() << "," << eye.z() << ") center=(" << center.x() << "," << center.y() << "," << center.z() << ") up=(" << up.x() << "," << up.y() << "," << up.z() << ")" << std::endl;

            const osg::Viewport *viewport = (*iter)->getViewport();
            if (viewport)
                std::cout << "   Camera viewport:    " << viewport->x() << "," << viewport->y() << " " << viewport->width() << "x" << viewport->height() << std::endl;
            else
                std::cout << "   Camera viewport:    INVALID" << std::endl;

			osg::View *v = (*iter)->getView();
			std::cout << "   view numSlaves:   " << v->getNumSlaves() << std::endl;

            for (unsigned int slaveNum=0; slaveNum<v->getNumSlaves(); slaveNum++)
            {
                osg::Camera *slaveCam = v->getSlave(slaveNum)._camera.get();
                std::cout << "   Slave '" << slaveCam->getName() << "':" << std::endl;
            
			    osg::Vec4 v4 = slaveCam->getClearColor();
			    std::cout << "     clear color:      (" << v4.x() << "," << v4.y() << "," << v4.z() << "," << v4.w() << ")" << std::endl;
			
			    slaveCam->getProjectionMatrixAsFrustum (left, right, bottom, top, zNear, zFar);
			    std::cout << "     Frustum:          " << left << "," << right << "," << top << "," << bottom << "  clip: " << zNear << "-" << zFar << std::endl;

			    slaveCam->getViewMatrixAsLookAt (eye, center, up);
			    std::cout << "     Camera LookAt:    eye=(" << eye.x() << "," << eye.y() << "," << eye.z() << ") center=(" << center.x() << "," << center.y() << "," << center.z() << ") up=(" << up.x() << "," << up.y() << "," << up.z() << ")" << std::endl;

			    const osg::Viewport *viewport = slaveCam->getViewport();
			    if (viewport)
			        std::cout << "     Camera viewport:  pos=(" << viewport->x() << "," << viewport->y() << ") size=" << viewport->width() << "x" << viewport->height() << std::endl;
			    else
			        std::cout << "     Camera viewport:  INVALID" << std::endl;
            }
		}
	}

	
	// *************************************************************************
	// any option left unread are converted into errors to write out later.
	arguments.reportRemainingOptionsAsUnrecognized();

	// report any errors if they have occured when parsing the program aguments.
	if (arguments.errors())
	{
		arguments.writeErrorMessages(std::cout);
		return 1;
	}

	// *************************************************************************
	// set up any initial scene elements:

	if (argScene.valid()) {
		std::cout << "Loading sample model" << std::endl;
		spin.sceneManager->worldNode->addChild(argScene.get());
	}

	// *************************************************************************
	// start threads:
	viewer.realize();

    // Try to subscribe with the current (default or manually specified) TCP
    // subscription information. If this fails, it's likely because the server
    // is not online, but when it comes online, it will send a userRefresh
    // message which will invoke another subscription attempt:
    spinListener.subscribe();

	// ask for refresh:
	spin.SceneMessage("s", "refresh", LO_ARGS_END);

	osg::Timer_t lastFrameTick = osg::Timer::instance()->tick();

	double minFrameTime = 1.0 / maxFrameRate;


	//std::cout << "Starting viewer (threading = " << viewer.getThreadingModel() << ")" << std::endl;
    std::cout << "\nspinviewer is READY" << std::endl;

	// program loop:
	while(not viewer.done())
	{
		
		if (spinListener.isRunning())
		{
			// ***** ORIGINAL (pollUpdates is done in clientContext thread):
			/*
			osg::Timer_t startFrameTick = osg::Timer::instance()->tick();

			spinListener.pollUpdates();
			
			pthread_mutex_lock(&sceneMutex);
			viewer.frame();
			pthread_mutex_unlock(&sceneMutex);

			if (maxFrameRate>0)
			{
				// work out if we need to force a sleep to hold back the frame rate
				osg::Timer_t endFrameTick = osg::Timer::instance()->tick();
				double frameTime = osg::Timer::instance()->delta_s(startFrameTick, endFrameTick);
				if (frameTime < minFrameTime) OpenThreads::Thread::microSleep(static_cast<unsigned int>(1000000.0*(minFrameTime-frameTime)));
			}
			*/

			// ***** NEW (pollUpdates are done in same thread as viewer)

			double dt = osg::Timer::instance()->delta_s(lastFrameTick, osg::Timer::instance()->tick());

			if (dt >= minFrameTime)
			{
				// we used to just call viewer.frame() within a mutex, but we
				// only really need to apply the mutex to the update traversal
				/*
				pthread_mutex_lock(&sceneMutex);
				viewer.frame();
				pthread_mutex_unlock(&sceneMutex);
				*/

				viewer.advance();
				viewer.eventTraversal();
				pthread_mutex_lock(&sceneMutex);
				spin.sceneManager->update();
                viewer.updateTraversal();
				viewer.renderingTraversals();
				pthread_mutex_unlock(&sceneMutex);
				
				// save time when the last time a frame was rendered:
				lastFrameTick = osg::Timer::instance()->tick();
				dt = 0;
			}

			unsigned int sleepTime;
			if (!recv) sleepTime = static_cast<unsigned int>(1000000.0*(minFrameTime-dt));
			else sleepTime = 0;
			if (sleepTime > 100) sleepTime = 100;

			if (!recv) OpenThreads::Thread::microSleep(sleepTime);

			// ***** END
		
		} else {
			
            for (int i=0; i<viewer.getNumViews(); i++)
            {
                // this should automatically release the manipulator (right??)
                viewer.getView(i)->setCameraManipulator(NULL);
            }

            // just in case, we'll check if the manipulator is still around:
			if (manipulator.valid())
			{
                manipulator.release();
			}
			
			viewer.setDone(true);
		}
	}

    // make sure we're done in case we didn't quit via interrupt
	spinListener.stop();

    return 0;
}

// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
int main(int argc, char **argv)
{
    /*
    // *************************************************************************
    // If no command line arguments were passed, check if there is an args file
    // at ~/.spinFramework/args and override argc and argv with those:
    std::vector<char*> newArgs;
    if (argc == 1)
    {
        try
        {
            using namespace boost::filesystem;
            
            if (exists(SPIN_DIRECTORY+"/args"))
            {
                std::stringstream ss;
                ss << std::ifstream( (SPIN_DIRECTORY+"/args").c_str() ).rdbuf();
                
                // executable path is always the first argument:
                newArgs.push_back(argv[0]);
                
                std::string token;
                while (ss >> token)
                {
                    char *arg = new char[token.size() + 1];
                    copy(token.begin(), token.end(), arg);
                    arg[token.size()] = '\0';
                    newArgs.push_back(arg);
                }
                newArgs.push_back(0); // needs to end with a null item
                
                argc = (int)newArgs.size()-1;
                argv = &newArgs[0];
            }
        }
        catch ( const boost::filesystem::filesystem_error& e )
        {
            std::cout << "Warning: cannot read arguments from " << SPIN_DIRECTORY+"/args. Reason: " << e.what() << std::endl;
        }
    }
     */
    
    
    // *************************************************************************
    // If no command line arguments were passed, check if there is an args file
    // at ~/.spinFramework/args and override argc and argv with those:
    std::vector<char*> newArgs = spin::getUserArgs();
    if ((argc==1) && (newArgs.size() > 1))
    {
        // need first arg (command name):
        newArgs.insert(newArgs.begin(), argv[0]);
        argc = (int)newArgs.size()-1;
        argv = &newArgs[0];
    }
    
    
    try
    {
        int result = run(argc, argv);
        std::cout << "\nspinviewer exited normally." << std::endl;
        return result;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Got exception " << e.what() << std::endl;
        return 1;
    }
}
