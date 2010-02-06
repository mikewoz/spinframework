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

#include <osgDB/ReadFile>
#include <osgDB/FileUtils>
#include <osg/Geometry>
#include <osg/Billboard>


#include "osgUtil.h"
#include "ShapeNode.h"
#include "SceneManager.h"
#include "MediaManager.h"

#include "VideoTexture.h"
#include "SharedVideoTexture.h"


using namespace std;


extern pthread_mutex_t pthreadLock;


// ===================================================================
// constructor:
ShapeNode::ShapeNode (SceneManager *sceneManager, char *initID) : GroupNode(sceneManager, initID)
{
	this->setName(string(id->s_name) + ".ShapeNode");
	nodeType = "ShapeNode";

	_color = osg::Vec4(1.0,1.0,1.0,1.0);	

	shape = NONE; //"NULL";
	billboard = RELATIVE; // ie, no billboard
	texturePath = "NULL";
	renderBin = 11;
	lightingEnabled = true;

}

// ===================================================================
// destructor
ShapeNode::~ShapeNode()
{
	if (sceneManager->sharedStateManager.valid()) sceneManager->sharedStateManager->prune();
}

// ===================================================================
// ======================== SET METHODS: =============================
// ===================================================================

void ShapeNode::setContext (const char *newvalue)
{
	// need to redraw after setContext() is called:
	ReferencedNode::setContext(newvalue);
	drawShape();
}

void ShapeNode::setShape (shapeType t)
{
	// don't do anything if the new shape is the same as the current shape:
	if (t == shape) return;
	else shape = t;

	//std::cout << "GOT NEW SHAPE MESSAGE: " << s << std::endl;

	drawShape();

	BROADCAST(this, "si", "setShape", (int) shape);

}

void ShapeNode::setBillboard (billboardType t)
{
	if (t == billboard) return;
	else billboard = t;

	drawShape();

	BROADCAST(this, "si", "setBillboard", (int) billboard);

}

void ShapeNode::setColor (float r, float g, float b, float a)
{
	osg::Vec4 newColor = osg::Vec4(r,g,b,a);

	if (_color != newColor)
	{
		_color = newColor;
	
		drawShape();
	
		BROADCAST(this, "sffff", "setColor", r, g, b, a);
	}
}

// ===================================================================
void ShapeNode::setTextureFromFile (const char* s)
{
	string path = getRelativePath(string(s));
	
	// don't do anything if the current texture is already loaded:
	if (path==texturePath) return;
	else texturePath=path;
	
	drawTexture();

	BROADCAST(this, "ss", "setTextureFromFile", texturePath.c_str());
}

// ===================================================================
void ShapeNode::setRenderBin (int i)
{
	renderBin = i;

	//std::cout << "TODO: fix renderbin.  " << renderBin << std::endl;


	if (shapeGeode.valid())
	{
		osg::StateSet *ss = shapeGeode->getOrCreateStateSet();
		ss->setRenderBinDetails( (int)renderBin, "RenderBin");
		//setStateSet( shapeStateSet );
	}

	BROADCAST(this, "si", "setRenderBin", renderBin);
}

void ShapeNode::setLighting (int i)
{

	lightingEnabled = (bool)i;

	if (shapeGeode.valid())
	{
		osg::StateSet *ss = shapeGeode->getOrCreateStateSet();
		if (lightingEnabled) ss->setMode( GL_LIGHTING, osg::StateAttribute::ON );
		else ss->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
	}

	BROADCAST(this, "si", "setLighting", getLighting());

}

// ===================================================================
void ShapeNode::drawShape()
{
    pthread_mutex_lock(&pthreadLock);

	// remove the old shape:
	if (this->getAttachmentNode()->containsNode(shapeGeode.get()))
	{
		this->getAttachmentNode()->removeChild(shapeGeode.get());
		shapeGeode = NULL;
	}

	bool ignoreOnThisHost = (sceneManager->isSlave() && (this->getContext()==getHostname()));

	//std::cout << "ShapeNode " << this->id->s_name << " ignore? " << (int)ignoreOnThisHost << std::endl;


	// TODO: this should only be added if this application is a graphical renderer.
	// There is no point to actually add the memory of the ShapeDrawable for apps
	// that do not need to use it!
	// ACTUALLY, the server needs to know about the geometry to compute 
	// intersections, determine bounding regions, radius, etc...
	if (shape && !ignoreOnThisHost)
	{

		osg::TessellationHints* hints = new osg::TessellationHints;
		hints->setDetailRatio(GENERIC_SHAPE_RESOLUTION);

		if (billboard)
		{
			osg::Billboard *b = new osg::Billboard();
			switch (billboard)
			{
				case POINT_EYE:
					b->setMode(osg::Billboard::POINT_ROT_EYE);
					break;
				case STAY_UP:
					b->setMode(osg::Billboard::AXIAL_ROT);
					b->setAxis(osg::Vec3(0.0f,0.0f,1.0f));
					b->setNormal(osg::Vec3(0.0f,1.0f,0.0f));
					break;
			}
			shapeGeode = b;
			
		} else {
			shapeGeode = new osg::Geode();
		}



		if (shape==PLANE) // OSG doesn't support planes
		{
			shapeGeode->addDrawable(createPlane(AS_UNIT_SCALE * .5, _color));
		}

		else {
			osg::ShapeDrawable *shapeDrawable;
			if (shape==SPHERE)
			{
				shapeDrawable = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.0f,0.0f,0.0f),AS_UNIT_SCALE*.5), hints);
			}
			else if (shape==BOX)
			{
				shapeDrawable = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0.0f,0.0f,0.0f), AS_UNIT_SCALE), hints);
			}
			else if (shape==CYLINDER)
			{
				shapeDrawable = new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3(0.0f,0.0f,0.0f), AS_UNIT_SCALE*.25,AS_UNIT_SCALE), hints);
			}
			else if (shape==CAPSULE)
			{
				shapeDrawable = new osg::ShapeDrawable(new osg::Capsule(osg::Vec3(0.0f,0.0f,0.0f), AS_UNIT_SCALE*.25,AS_UNIT_SCALE), hints);
			}
			else if (shape==CONE)
			{
				shapeDrawable = new osg::ShapeDrawable(new osg::Cone(osg::Vec3(0.0f,0.0f,0.0f), AS_UNIT_SCALE*.25,AS_UNIT_SCALE), hints);
			}
			else {
				return;
			}
			shapeGeode->addDrawable(shapeDrawable);
			shapeDrawable->setColor(_color);
		}




		/*
		switch (shape)
		{
			case SPHERE:
				shapeGeode->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.0f,0.0f,0.0f),AS_UNIT_SCALE*.5), hints));
				break;
			case BOX:
				shapeGeode->addDrawable(new osg::ShapeDrawable(new osg::Box(osg::Vec3(0.0f,0.0f,0.0f), AS_UNIT_SCALE), hints));
				break;
			case CYLINDER:
				shapeGeode->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3(0.0f,0.0f,0.0f), AS_UNIT_SCALE*.25,AS_UNIT_SCALE), hints));
				break;
			case CAPSULE:
				shapeGeode->addDrawable(new osg::ShapeDrawable(new osg::Capsule(osg::Vec3(0.0f,0.0f,0.0f), AS_UNIT_SCALE*.25,AS_UNIT_SCALE), hints));
				break;
			case CONE:
				shapeGeode->addDrawable(new osg::ShapeDrawable(new osg::Cone(osg::Vec3(0.0f,0.0f,0.0f), AS_UNIT_SCALE*.25,AS_UNIT_SCALE), hints));
				break;
			case PLANE:
				shapeGeode->addDrawable(createPlane(AS_UNIT_SCALE * .5));
				break;
			default:
				shapeGeode = NULL;
		}
		*/

	}


	if (shapeGeode.valid())
	{
		// wireframe test
		//osgFX::Scribe* scribe = new osgFX::Scribe();
		//modelGroup->addChild(scribe);
		//scribe->addChild(shapeGeode.get());
		// end wireframe test
		
		osg::StateSet *ss = shapeGeode->getOrCreateStateSet();
		ss->setMode( GL_BLEND, osg::StateAttribute::ON );
		ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

		if (lightingEnabled) ss->setMode( GL_LIGHTING, osg::StateAttribute::ON );
		else ss->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
		
		this->getAttachmentNode()->addChild(shapeGeode.get());
		shapeGeode->setName(string(id->s_name) + ".shapeGeode");
		optimizer.optimize(shapeGeode.get()); // ?
	}

	pthread_mutex_unlock(&pthreadLock);

	
	drawTexture();
	
}


// ===================================================================
void ShapeNode::drawTexture()
{
	

	//std::cout << "debug texturepath: " << texturePath <<  std::endl;


	if (texturePath==string("NULL"))
	{
		// remove current texture
		//shapeGeode->setStateSet( new osg::StateSet() );
		if (sceneManager->sharedStateManager.valid()) sceneManager->sharedStateManager->prune();
	}

	else if (shapeGeode.valid())
	{
		pthread_mutex_lock(&pthreadLock);
		
		std::string fullPath = getAbsolutePath(texturePath);
		
		// if filename contains "shared_video_texture", then replace
		// current TextureAttribute with a SharedVideoTexture
		
		size_t pos;

		if ((pos=fullPath.find("shared_video_texture")) != string::npos)
		{
			// find the shared memory id from the filename:
			std::string shID = "shvid_"+fullPath.substr(pos+20, fullPath.rfind(".")-(pos+20));
			osg::ref_ptr<SharedVideoTexture> shvid = dynamic_cast<SharedVideoTexture*>(sceneManager->getOrCreateState(shID.c_str(), "SharedVideoTexture"));
			if (shvid.valid())
			{
				shapeGeode->setStateSet( shvid.get() );
			} else {
				std::cout << "ERROR creating SharedVideoTexture '" << shID << "' for ShapeNode: " << id->s_name << std::endl;
			}

		}

		else if (isVideoPath(fullPath))
		{
			osg::ref_ptr<VideoTexture> vid = dynamic_cast<VideoTexture*>(sceneManager->getOrCreateState((string(id->s_name)+"/VideoTexture").c_str(), "VideoTexture"));
			if (vid.valid())
			{
				vid->setVideoPath(fullPath.c_str());
				shapeGeode->setStateSet( vid.get() );
			} else {
				std::cout << "ERROR creating VideoTexture '" << texturePath << "' for ShapeNode: " << id->s_name << std::endl;
			}
		}
		
		else
		{
			addImageTexture(shapeGeode.get(), fullPath);
		}

		pthread_mutex_unlock(&pthreadLock);
	}
}
			
			
void ShapeNode::addImageTexture(osg::Node *n, std::string path)
{
	if (!sceneManager->isGraphical()) return;
	
	osg::StateSet *ss = n->getOrCreateStateSet();
		
	//osg::ref_ptr<osg::Image> image;
	osg::ref_ptr<osg::Image> textureImage = osgDB::readImageFile( path.c_str() );
	if (textureImage.valid())
	{

		// old:
		/*
		shapeStateSet->setMode( GL_BLEND, osg::StateAttribute::ON );
		osg::Texture2D* shapeTexture = new osg::Texture2D();
		shapeTexture->setBorderColor(osg::Vec4(1.0f,1.0f,1.0f,0.0f));
		shapeTexture->setImage(textureImage.get());
		shapeStateSet->setTextureAttributeAndModes(0,shapeTexture,osg::StateAttribute::ON);
		shapeStateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
		*/


		osg::Texture2D* shapeTexture = new osg::Texture2D();
		shapeTexture->setBorderColor(osg::Vec4(1.0f,1.0f,1.0f,0.0f));
		shapeTexture->setImage(textureImage.get());

		//osg::StateSet *ss = new osg::StateSet();
		//osg::StateSet *ss = shapeGeode->getOrCreateStateSet();

		// Turn blending on:
		ss->setMode( GL_BLEND, osg::StateAttribute::ON );
		ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

		// Disable depth testing so geometry is drawn regardless of depth values
		// of geometry already draw.
		//ss->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);

		// Disable lighting:
		//ss->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

		// Need to make sure this geometry is draw last. RenderBins are handled
		// in numerical order.
		ss->setRenderBinDetails( renderBin, "RenderBin");

		// Set Texture:
		ss->setTextureAttributeAndModes(0,shapeTexture,osg::StateAttribute::ON);

		//n->setStateSet( shapeStateSet );

		if (sceneManager->sharedStateManager.valid())
				sceneManager->sharedStateManager->share(n);

	} else {
		std::cout << "ERROR (setTexture): The file " << path << " is not a valid texture." << std::endl;
	}


}

std::vector<lo_message> ShapeNode::getState ()
{
	// inherit state from base class
	std::vector<lo_message> ret = GroupNode::getState();

	lo_message msg;
	osg::Vec3 v;
	osg::Vec4 v4;

	//std::cout << "in getState. shape=" << shape << ", textureName=" << textureName << std::endl;

	msg = lo_message_new();
	lo_message_add(msg, "si", "setBillboard", getBillboard());
	ret.push_back(msg);
	
	msg = lo_message_new();
	v4 = this->getColor();
	lo_message_add(msg, "sffff", "setColor", v4.x(), v4.y(), v4.z(), v4.w());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg,  "ss", "setTextureFromFile", texturePath.c_str());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "si", "setRenderBin", getRenderBin());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "si", "setLighting", getLighting());
	ret.push_back(msg);

	// put this one last:	
	msg = lo_message_new();
	lo_message_add(msg, "si", "setShape", getShape());
	ret.push_back(msg);



	return ret;
}
