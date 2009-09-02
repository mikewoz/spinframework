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

#include <iostream>
#include <osg/Geometry>
#include <osgViewer/CompositeViewer>
#include <osgViewer/View>
#include <osg/GraphicsContext>
#include <osgGA/NodeTrackerManipulator>
#include <osgGA/TrackballManipulator>
#include <osgViewer/ViewerEventHandlers>
#include <osg/TextureCubeMap>
#include <osg/Image>

#include "util.h"
#include "asCameraManager.h"
#include "asCameraView.h"
#include "tinyxml.h"
#include "tinystr.h"


using namespace std;

using namespace osgViewer;

asCameraManager::asCameraManager (string viewerID, osgViewer::CompositeViewer &vr, string listenAddr, string listenPort)
{
	initialized = false;
	
	id = viewerID;
	viewer = &vr;

	// start a OSC listener thread on port:
	if (isMulticastAddress(listenAddr))
	{
		oscListener = lo_server_thread_new_multicast(listenAddr.c_str(), listenPort.c_str(), oscListener_error);
	} else {
		oscListener = lo_server_thread_new(listenPort.c_str(), oscListener_error);
	}
	
	
	int sock = lo_server_get_socket_fd(lo_server_thread_get_server(oscListener));
	//int sockopt = 1;
	//if (setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, &sockopt, sizeof(sockopt)))
	//	std::cout << "cameraManager ERROR: Could not set SO_REUSEPORT flag." << std::endl;
	
	lo_server_thread_add_method(oscListener, string("/asViewer/"+id).c_str(), NULL, oscListener_admin_callback, this);
	lo_server_thread_start(oscListener);

	std::cout << "  asCameraManager '" << id << "' is listening on " << listenAddr << ", port: " << lo_server_thread_get_port(oscListener) << std::endl;

	
	cameraList.clear();

	cameraGroup = new osg::Group();
	
}

asCameraManager::~asCameraManager()
{
	lo_server_thread_free(oscListener);
	
	// remove all camera nodes from scenegraph
	vector< osg::ref_ptr<asCameraView> >::iterator iter;
	for (iter = cameraList.begin(); iter!=cameraList.end(); iter++)
	{
		if (cameraGroup->containsNode((*iter).get()))
		{
			cameraGroup->removeChild((*iter).get());
		}
	}
	
	cameraList.clear();
}

void asCameraManager::debugPrint()
{
	osg::Vec3 v3;
	osg::Vec4 v4;
	osg::Vec3 eye, center, up;
	float dist;
	double left, right, bottom, top, zNear, zFar;

	std::cout << "\n***************************************" << std::endl;
	std::cout <<   "******** asCameraManager info *********" << std::endl;

	
	vector< osg::ref_ptr<asCameraView> >::iterator iter;
	for (iter = cameraList.begin(); iter!=cameraList.end(); iter++)
	{
		std::cout.precision(3);

		
		std::cout << "Camera '" << (*iter)->getID() << "':" << std::endl;
		
		std::cout << "  scene:            " << (*iter)->getView()->getSceneData()->getName() << std::endl;
		
		std::cout << "  distortion mode:  " << (*iter)->getDistortionType() << std::endl;
		
		v3 = (*iter)->getPosition();
		std::cout << "  cam.translation:  (" << v3.x() << "," << v3.y() << "," << v3.z() << ")" << std::endl;
		v3 = (*iter)->getAttitude().asVec3();
		std::cout << "  cam.orientation:  (" << osg::RadiansToDegrees(v3.x()) << "," << osg::RadiansToDegrees(v3.y()) << "," << osg::RadiansToDegrees(v3.z()) << ")" << std::endl;
		
		std::cout << "  GL drawBuffer:    " << (*iter)->getView()->getCamera()->getDrawBuffer() << std::endl;
		std::cout << "  GL readBuffer:    " << (*iter)->getView()->getCamera()->getReadBuffer() << std::endl;

		(*iter)->getView()->getCamera()->getProjectionMatrixAsFrustum (left, right, bottom, top, zNear, zFar);
		std::cout << "  Frustum:          " << left << "," << right << "," << top << "," << bottom << "  clip: " << zNear << "-" << zFar << std::endl;

		const osg::Viewport *vwport = (*iter)->getView()->getCamera()->getViewport();
		std::cout << "  Viewport:         (pos=" << vwport->x() << "," << vwport->y() << "  size=" << vwport->width() << "," << vwport->height() << ")" << std::endl;
	
		v4 = (*iter)->getView()->getCamera()->getClearColor();
		std::cout << "  Viewport color:   (" << v4.x() << "," << v4.y() << "," << v4.z() << "," << v4.w() << ")" << std::endl;
	
		
		(*iter)->getView()->getCamera()->getViewMatrixAsLookAt (eye, center, up, dist);
		std::cout << "  Camera LookAt:    eye=(" << eye.x() << "," << eye.y() << "," << eye.z() << ") center=(" << center.x() << "," << center.y() << "," << center.z() << ") up=(" << up.x() << "," << up.y() << "," << up.z() << ")" << std::endl;
		
		osg::View::LightingMode lm = (*iter)->getView()->getLightingMode();
		if (lm==osg::View::NO_LIGHT) std::cout <<  "  LightingMode:     NO_LIGHT" << std::endl;
		if (lm==osg::View::SKY_LIGHT) std::cout << "  LightingMode:     SKY_LIGHT" << std::endl;
		if (lm==osg::View::HEADLIGHT) std::cout << "  LightingMode:     HEADLIGHT" << std::endl;

		
		int numSlaves = (*iter)->getView()->getNumSlaves();
		std::cout << "  Slave cameras:    " << numSlaves << std::endl;
		
		
		for (int i=0; i<numSlaves; i++)
		{
			osg::View::View::Slave slv = (*iter)->getView()->getSlave(i);
			osg::Camera *slaveCam = slv._camera.get();
			std::cout << "    slave["<<i<<"]: " << slaveCam->getName() << std::endl;
				std::cout << "      IsRenderToTexture? " << slaveCam->isRenderToTextureCamera() << std::endl;
			const osg::Viewport *slvport = slaveCam->getViewport();
			std::cout << "      Viewport:         (pos=" << slvport->x() << "," << slvport->y() << "  size=" << slvport->width() << "," << slvport->height() << ")" << std::endl;
			//slv._viewOffset
			//std::cout << "      Camera view offset: "   << std::endl;
			slaveCam->getViewMatrixAsLookAt (eye, center, up, dist);
			std::cout << "      Camera LookAt:    eye=(" << fixed << eye.x() << "," << eye.y() << "," << eye.z() << ") center=(" << center.x() << "," << center.y() << "," << center.z() << ") up=(" << up.x() << "," << up.y() << "," << up.z() << ")" << std::endl;
					
		}
		
		
		osg::ref_ptr<asWindow> win = (*iter)->getWindow();
		std::cout << "  Window '" << win->getID() << "':" << std::endl;
		std::cout << "    displayNum: " << win->getGfxTraits()->displayNum << ", screenNum: " << win->getGfxTraits()->screenNum << std::endl;
		std::cout << "    pos= " << win->getGfxTraits()->x << "," << win->getGfxTraits()->y << std::endl;
		std::cout << "    size= " << win->getGfxTraits()->width << "," << win->getGfxTraits()->height << std::endl;
		
	}
	
	
	std::vector<osg::GraphicsContext*> allContexts;
	viewer->getContexts(allContexts);
	std::cout << "TOTAL GraphicsContexts: " << allContexts.size() << std::endl;
	
}



void asCameraManager::refreshWindows()
{
	vector< osg::ref_ptr<asWindow> >::iterator windowIter;
	for (windowIter = windowList.begin(); windowIter!=windowList.end(); windowIter++)
	{
		const osg::GraphicsContext::Traits *gfxTraits = (*windowIter)->getGfxTraits();
		(*windowIter)->getGfxContext()->resized(gfxTraits->x, gfxTraits->y, gfxTraits->width, gfxTraits->height);
	}
	
	vector< osg::ref_ptr<asCameraView> >::iterator cameraIter;
	for (cameraIter = cameraList.begin(); cameraIter!=cameraList.end(); cameraIter++)
	{
		// refresh viewport:
		(*cameraIter)->updateViewport();
	}
	
	
}


asWindow *asCameraManager::getWindow(int id)
{
	vector< osg::ref_ptr<asWindow> >::iterator iter;
	for (iter = windowList.begin(); iter!=windowList.end(); iter++)
	{
		if ((*iter)->getID()==id)
		{
			return (*iter).get();
			break;
		}
	}
	return NULL;
}


asCameraView *asCameraManager::getCamera(const char *id)
{
	// first, we need to find it in the cameraList:
	vector< osg::ref_ptr<asCameraView> >::iterator iter;
	for (iter = cameraList.begin(); iter!=cameraList.end(); iter++)
	{
		if (string((*iter)->getID())==string(id))
		{
			return (*iter).get();
			break;
		}
	}
	return NULL;
}

void asCameraManager::createCamera(const char *id)
{
	createCamera(id, 0);
}

void asCameraManager::createCamera(const char *id, int windowID)
{
	// Check if windowNum is valid. If not, create that window:
	osg::ref_ptr<asWindow> win = getWindow(windowID);
	if (!win.valid())
	{
		if (!initialized)
		{
			// we can still create windows
			win = new asWindow(windowID);
			windowList.push_back(win);
		} else {
			std::cout << "ERROR: Tried to create camera " << id << " on window " << windowID << ", but that window does not exist and cannot be create dynamically during runtime. Please create all windows during initialization." << std::endl;
			return;
		}
	}	
	
	osg::ref_ptr<asCameraView> cam = new asCameraView(id, *viewer, win);
	
	if (cam.valid())
	{
		cameraList.push_back(cam);
		
		string OSCpath = "/asViewer/" + this->id + "/" + string(id);
		lo_server_thread_add_method(oscListener, OSCpath.c_str(), NULL, oscListener_callback, cam.get());
		
	} else {
		std::cout << "ERROR: Failed to create caneraView (" << id << ")" << std::endl;
	}
	
	
	// add the cameraView to the scene (it's just a PAT), so that it can be used
	// as a nodeTracker for the camera it references:
	cameraGroup->addChild(cam.get());
	
}

void asCameraManager::destroyCamera(const char *id)
{
	// first, we need to find it in the cameraList:
	vector< osg::ref_ptr<asCameraView> >::iterator iter;
	for (iter = cameraList.begin(); iter!=cameraList.end(); iter++)
	{
		if (string((*iter)->getID())==string(id))
		{
			// remove it from the scene:
			if (cameraGroup->containsNode((*iter).get())) cameraGroup->removeChild((*iter).get());
			
			// remove the OSC method:
			string oscPattern = "/asViewer/" + this->id + "/" + string(id);
			lo_server_thread_del_method(oscListener, oscPattern.c_str(), NULL);
			
			// now remove it from the cameraList:
			cameraList.erase(iter);
			
			// now all osg::ref_ptr references should be gone, and OSG will do the rest
			
			break;
		}
	}
	
}


void asCameraManager::sendCameraList(lo_address addr, lo_server serv)
{
	lo_message msg = lo_message_new();
	
	vector< osg::ref_ptr<asCameraView> >::iterator iter;
	lo_message_add_string(msg, "cameraList");
	for (iter = cameraList.begin(); iter!=cameraList.end(); iter++)
	{
		lo_message_add_string(msg, (char*)(*iter)->getID());
	}
	
	//lo_message_pp(msg);
	
	string OSCpath = "/asViewer/" + this->id;
	if (serv) lo_send_message_from(addr, serv, OSCpath.c_str(), msg);

	else lo_message_free(msg);
}


void asCameraManager::setResolution(std::string resolutionString)
{
	// assume that we want to set the resolution for all windows:
	vector< osg::ref_ptr<asWindow> >::iterator iter;
	for (iter = windowList.begin(); iter!=windowList.end(); iter++)
	{
		(*iter)->setResolution(resolutionString);
	}
}


void asCameraManager::setSceneData(osg::Node *node)
{

	// should we try to remove the old node (if this was already done)?

	/*
	vector< osg::ref_ptr<asCameraView> >::iterator iter;
	for (iter = cameraList.begin(); iter!=cameraList.end(); iter++)
	{
		cameraGroup->addChild(node);
	}
	*/
	
	// We add the scene to the same node to which all the cameraViews are
	// attached. This means that the scene and the PositionAttitudeTransforms
	// for a cameraView are in the same coordinate system.
	
	sceneRoot = node;
	
	//if (!cameraGroup->containsNode(sceneRoot.get())) cameraGroup->addChild(sceneRoot.get());
	
	
}


void asCameraManager::init()
{

	if (!cameraList.size())
	{
		// create at least one camera:
		this->createCamera("default", 0);
	}

	this->refreshWindows();
	
	
	// call the update() method once, in order to attach the views:
	this->update();
	

	this->initialized = true;

}

void asCameraManager::update()
{
	vector< osg::ref_ptr<asCameraView> >::iterator iter;
	for (iter = cameraList.begin(); iter!=cameraList.end(); iter++)
	{
		if ((*iter)->removalFlag)
		{
			viewer->removeView((*iter)->getView());
			(*iter)->removalFlag = false;
		}
		
		if ((*iter)->additionFlag)
		{
			viewer->addView((*iter)->getView());
			(*iter)->getView()->setSceneData(sceneRoot.get());
			
			(*iter)->additionFlag = false;	
		}
		
		if ((*iter)->distortionUpdate)
		{
			//viewer->removeView((*iter)->getView());

			if ((*iter)->getDistortionType()=="NONE") (*iter)->makePlanar();
			else if ((*iter)->getDistortionType()=="DOME") (*iter)->makeDome();
			(*iter)->distortionUpdate = false;
			
			//viewer->addView((*iter)->getView());
			(*iter)->getView()->setSceneData(sceneRoot.get());
		}
		
	}

}
/*
void asCameraManager::loadXML(std::string configFile)
{	
	TiXmlElement *winElem = 0;
	TiXmlElement *camElem = 0;
	TiXmlElement *n = 0;
	string tag="", val="";
	float v[4];
	osg::ref_ptr<asCameraView> cam;
	osg::ref_ptr<osg::GraphicsContext::Traits> traits;
	

	// first check if the wsi is valid:
	osg::GraphicsContext::WindowingSystemInterface* wsi = osg::GraphicsContext::getWindowingSystemInterface();
	if (!wsi)
	{
		osg::notify(osg::NOTICE)<<"Error, no WindowSystemInterface available, cannot create windows."<<std::endl;
		return;
	}
	
	
	TiXmlDocument doc( configFile.c_str() );
	TiXmlHandle docHandle( &doc );

	// Load the XML file and verify:
	if ( !doc.LoadFile() ) {
		std::cout << "\nWARNING: failed to load " << configFile << ". Invalid XML format." << std::endl;
		return;
	}
	
	else {
		// verify there is at least one window tag with at least one camera:
		n = docHandle.FirstChild("OSGconfig").FirstChild("window").FirstChild("camera").ToElement();
		if (!n)
		{
			std::cout << "\nWARNING: failed to load " << configFile << ". There must be at least one window and camera defined." << std::endl;
			return;
		}
	}

	

	// okay.. we have a valid xml file.
	std::cout << "Loading camConfig file (" << configFile << ")" << std::endl;
				
	// look for windows:
	winElem = docHandle.FirstChild("OSGconfig").FirstChild("window").ToElement();
	for ( winElem; winElem; winElem = winElem->NextSiblingElement("window") )
	{

	
		// create a GraphicsContext::Traits for this window and initialize with some defaults:
		traits = new osg::GraphicsContext::Traits;
		
		traits->x = 0;
		traits->y = 0;
		traits->width = 320;
		traits->height = 240;
		traits->windowDecoration = true;
		traits->doubleBuffer = true;
		traits->useCursor = true;
		traits->supportsResize = true;
		traits->sharedContext = 0;
		osg::Vec4 clearColor = osg::Vec4f(0.1f,0.1f,0.1f,1.0f);
	
		// note: can get hostname with: traits->hostName 
	
	
		// update traits parameters from xmlElement:
		if (n = winElem->FirstChildElement("screen"))
		{
			traits->screenNum = atoi(n->FirstChild()->Value());
		}
		
		// Now that we have the screen, let's get the resolution. This is important because
		// size and position in the xml file is specified from 0 to 1 (ie, fraction of screen)
		unsigned int maxWidth, maxHeight;
		wsi->getScreenResolution(osg::GraphicsContext::ScreenIdentifier(traits->screenNum), maxWidth, maxHeight);
	
		if (n = winElem->FirstChildElement("size"))
		{
			sscanf( n->FirstChild()->Value(), "%f %f", &v[0],&v[1]);
			traits->width = (int) (maxWidth * v[0]);
			traits->height = (int) (maxHeight * v[1]);
		}
	
		if (n = winElem->FirstChildElement("position"))
		{
			sscanf( n->FirstChild()->Value(), "%f %f", &v[0],&v[1]);
			traits->x = (int) (maxWidth * v[0]);
			traits->y = (int) (maxHeight * v[1]);
		}
	
		if (n = winElem->FirstChildElement("useCursor"))
		{
			if (n->FirstChild()->Value() == "false") traits->useCursor = false;
			else traits->useCursor = true;
		}
		
		if (n = winElem->FirstChildElement("windowDecoration"))
		{
			if (n->FirstChild()->Value() == "false") traits->windowDecoration = false;
			else traits->windowDecoration = true;
		}
		
		
		if (n = winElem->FirstChildElement("supportsResize"))
		{
			if (n->FirstChild()->Value() == "false") traits->supportsResize = false;
			else traits->supportsResize = true;
		}
	
		if (n = winElem->FirstChildElement("clearColor"))
		{
			sscanf( n->FirstChild()->Value(), "%f %f %f %f", &v[0],&v[1],&v[2],&v[3]);
			clearColor = osg::Vec4f(v[0],v[1],v[2],v[3]);
			std::cout << "clearColor is " << v[0] << v[1] << v[2] << v[3] << std::endl;
		}
	
	
		// okay, now create the graphicsContext
	
		osg::ref_ptr<osg::GraphicsContext> gfxContext = osg::GraphicsContext::createGraphicsContext(traits.get());
		if (gfxContext.valid())
		{
			gfxContext->setClearColor(clearColor);
			gfxContext->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			osg::notify(osg::INFO)<<"  GraphicsWindow has been created successfully."<<std::endl;
		}
		else
		{
			osg::notify(osg::NOTICE)<<"  GraphicsWindow has not been created successfully."<<std::endl;
		}
		
		// now search for cameras:
		for ( camElem = winElem->FirstChildElement("camera"); camElem; camElem = camElem->NextSiblingElement("camera") )
		{
	
			// create a view:
			if ( camElem->Attribute("id") )
			{
			
				cam = new asCameraView((char*) camElem->Attribute("id"), *viewer, gfxContext);
				cameraList.push_back(cam);
			
		
			} else {
				std::cout << "camConfig ERROR: camera must have id" << std::endl;
				break;
			}
	
			// get all properties of the camera:
			for ( n = camElem->FirstChildElement(); n; n = n->NextSiblingElement() )
			{
				// get tag and value:
				if (n->FirstChild())
				{
					tag = n->Value();
					val = n->FirstChild()->Value();
				} else continue;
				
				// now update camera parameters:
				if (tag=="translation")
				{
					if (sscanf (val.c_str(),"%f %f %f",&v[0],&v[1],&v[2]))
						cam->setTranslation(v[0],v[1],v[2]);
				}
				else if (tag=="orientation")
				{
					if (sscanf (val.c_str(),"%f %f %f",&v[0],&v[1],&v[2]))
						cam->setOrientation(v[0],v[1],v[2]);
				}
		
				else if (tag=="viewport")
				{
					if (sscanf (val.c_str(),"%f %f %f %f",&v[0],&v[1],&v[2],&v[3]))
					{
						cam->setViewport( (int) (v[0]*traits->width), (int) (v[1]*traits->height), (int) (v[2]*traits->width), (int) (v[3]*traits->height) );
					}
				}
				else if (tag=="clearColor")
				{
					if (sscanf (val.c_str(), "%f %f %f %f", &v[0],&v[1],&v[2],&v[3]))
					{
						cam->setClearColor(v[0],v[1],v[2],v[3]);
					}
				}
				
				else
				{
					std::cout << "Unknown parameter in configuration file: " << tag << std::endl;
				}
			}
		}
	}
}
*/






// *****************************************************************************

static int oscListener_admin_callback(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data)
{
	asCameraManager *cameraManager = (asCameraManager*) user_data;

	string    theMethod;	
	vector<float> floatArgs;
	vector<string> stringArgs;

	// make sure there is at least one argument (ie, a method to call):
	if (!argc) return 0;

	// get the method (argv[0]):
	if (lo_is_string_type((lo_type)types[0]))
	{
		theMethod = string((char *)argv[0]);
	}
	else return 0;
	
	// get the rest of the args:
	for (int i=1; i<argc; i++)
	{
		if (lo_is_numerical_type((lo_type)types[i]))
		{
			floatArgs.push_back( (float) lo_hires_val((lo_type)types[i], argv[i]) );
		} else {
			stringArgs.push_back( (char*) argv[i] );
		}
	}
	
	
	// create a new asCamera:
	if (theMethod=="debug")
	{
		cameraManager->debugPrint();
	}
	
	else if ((theMethod=="create") && (stringArgs.size()==1) && (floatArgs.size()==1))
	{
		cameraManager->createCamera((char*) stringArgs[0].c_str(), (int)floatArgs[0]);
	}
	
	else if ((theMethod=="create") && (stringArgs.size()==1))
	{
		cameraManager->createCamera((char*) stringArgs[0].c_str());
	}
	
	else if ((theMethod=="destroy") && (stringArgs.size()==1))
	{
		cameraManager->destroyCamera((char*) stringArgs[0].c_str());
	}
	
	else if ((theMethod=="setResolution") && (stringArgs.size()==1))
	{
		cameraManager->setResolution(stringArgs[0]);
	}
	
	else {
		std::cout << "Unknown OSC command: " << path << " " << theMethod << " (with " << argc-1 << " args)" << std::endl;
	}
	
	return 1;
}

static int oscListener_callback(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data)
{
	// check that we have a camera pointer:
	osg::ref_ptr<asCameraView> cam = (asCameraView*) user_data;
	if (!cam.valid()) return 0;
	
	if (0)
	{
		// debug print:
		printf("************ oscListener_callback() got message: %s\n", (char*)path);
		printf("user_data says it's for camera: %s\n", cam->getID());
		for (int i=0; i<argc; i++) {
			printf("arg %d '%c' ", i, types[i]);
	    	lo_arg_pp((lo_type) types[i], argv[i]);
	    	printf("\n");
		}
		printf("\n");
		fflush(stdout);
	}
	
	
	int i;
	string    theMethod;
	vector<float> floatArgs;
	vector<string> stringArgs;

	// make sure there is at least one argument (ie, a method to call):
	if (!argc) return 0;

	// get the method (argv[0]):
	if (lo_is_string_type((lo_type)types[0]))
	{
		theMethod = string((char *)argv[0]);
	}
	else return 0;
	
	// get the rest of the args:
	for (i=1; i<argc; i++)
	{
		if (lo_is_numerical_type((lo_type)types[i]))
		{
			floatArgs.push_back( (float) lo_hires_val((lo_type)types[i], argv[i]) );
		} else {
			stringArgs.push_back( (char*) argv[i] );
		}
	}
	
	// now apply the method:
	
	if ((theMethod=="setTranslation") && (floatArgs.size()==3))
	{
		cam->setTranslation(floatArgs[0], floatArgs[1], floatArgs[2]);
	}
	
	else if ((theMethod=="setOrientation") && (floatArgs.size()==3))
	{
		cam->setOrientation(floatArgs[0], floatArgs[1], floatArgs[2]);
	}
	
	else if ((theMethod=="move") && (floatArgs.size()==3))
	{
		cam->move(floatArgs[0], floatArgs[1], floatArgs[2]);
	}
	
	else if ((theMethod=="rotate") && (floatArgs.size()==3))
	{
		cam->rotate(floatArgs[0], floatArgs[1], floatArgs[2]);
	}
	
	else if ((theMethod=="setViewport") && (floatArgs.size()==4))
	{
		cam->setViewport(floatArgs[0], floatArgs[1], floatArgs[2], floatArgs[3]);
	}
	
	else if ((theMethod=="setFrustum") && (floatArgs.size()==6))
	{
		cam->setFrustum(floatArgs[0], floatArgs[1], floatArgs[2], floatArgs[3], floatArgs[4], floatArgs[5]);
	}
	
	else if ((theMethod=="setClearColor") && (floatArgs.size()==4))
	{
		cam->setClearColor(floatArgs[0], floatArgs[1], floatArgs[2], floatArgs[3]);
	}
	
	else if ((theMethod=="setDistortion") && (stringArgs.size()==1))
	{
		// check for good args:
		if ((stringArgs[0]=="DOME") && (floatArgs.size()==2))
		{
			cam->setDistortion(stringArgs[0], floatArgs);
		} else if (stringArgs[0]=="NONE")
		{
			cam->setDistortion(stringArgs[0], floatArgs);
		}
	}

	else if ((theMethod=="setResolution") && (stringArgs.size()==1))
	{
		cam->getWindow()->setResolution(stringArgs[0]);
	}


	
	else {
		
		std::cout << "Unknown OSC command: " << path << " " << theMethod << " (with " << argc-1 << " args)" << std::endl;
		
	}
		
	return 1;
}


static void oscListener_error(int num, const char *msg, const char *path)
{
	printf("OSC_Listener error %d in path %s: %s\n", num, path, msg);
	fflush(stdout);
}


