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
#include <osg/Geometry>

#include "osgUtil.h"
#include "ShapeNode.h"
#include "SceneManager.h"
#include "MediaManager.h"



using namespace std;


extern pthread_mutex_t pthreadLock;


// ===================================================================
// constructor:
ShapeNode::ShapeNode (SceneManager *sceneManager, char *initID) : ReferencedNode(sceneManager, initID)
{
	this->setName(string(id->s_name) + ".ShapeNode");
	nodeType = "ShapeNode";

	shapeTransform = new osg::PositionAttitudeTransform();
	shapeTransform->setName(string(id->s_name) + ".shapeTransform");
	this->addChild(shapeTransform.get());

	// When children are attached to this, they get added to the attachNode:
	// NOTE: by changing this, we MUST override the updateNodePath() method!
	setAttachmentNode(shapeTransform.get());

	_color = osg::Vec4(1.0,1.0,1.0,1.0);

	shape = NONE; //"NULL";
	texturePath = "NULL";
	renderBin = 11;

}

// ===================================================================
// destructor
ShapeNode::~ShapeNode()
{

}

// ===================================================================

// IMPORTANT:
// subclasses of ReferencedNode are allowed to contain complicated subgraphs, and
// can also change their attachmentNode so that children are attached anywhere
// in this subgraph. If that is the case, the updateNodePath() function MUST be
// overridden, and extra nodes must be manually pushed onto the currentNodePath.

void ShapeNode::updateNodePath()
{
	currentNodePath.clear();
	if ((parent!=WORLD_SYMBOL) && (parent!=NULL_SYMBOL))
	{
		osg::ref_ptr<ReferencedNode> parentNode = parent->s_thing;
		if (parentNode.valid())
		{
			currentNodePath = parentNode->currentNodePath;
		}
	}

	// here, the nodePath includes the base osg::group, PLUS the shapeTransform
	currentNodePath.push_back(this);
	currentNodePath.push_back(shapeTransform.get());

	// now update NodePaths for all children:
	updateChildNodePaths();

}

// ===================================================================
// ======================== SET METHODS: =============================
// ===================================================================


void ShapeNode::setShape (shapeType s)
{
	// don't do anything if the new shape is the same as the current shape:
	if (s == shape) return;
	else shape = s;

	//std::cout << "GOT NEW SHAPE MESSAGE: " << s << std::endl;

	drawShape();

	BROADCAST(this, "si", "setShape", (int) shape);

}

void ShapeNode::setColor (float r, float g, float b, float a)
{
	_color = osg::Vec4(r,g,b,a);

	drawShape();

	BROADCAST(this, "sffff", "setColor", r, g, b, a);
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


void ShapeNode::setTranslation (float x, float y, float z)
{
	shapeTransform->setPosition(osg::Vec3(x,y,z));

	BROADCAST(this, "sfff", "setTranslation", x, y, z);
}

// ===================================================================
void ShapeNode::setOrientation (float p, float r, float y)
{
	_orientation = osg::Vec3(p, r, y);

	osg::Quat q = osg::Quat( osg::DegreesToRadians(p), osg::Vec3d(1,0,0),
							 osg::DegreesToRadians(r), osg::Vec3d(0,1,0),
							 osg::DegreesToRadians(y), osg::Vec3d(0,0,1));

	shapeTransform->setAttitude(q);

	BROADCAST(this, "sfff", "setOrientation", p, r, y);
}

// ===================================================================
void ShapeNode::setScale (float x, float y, float z)
{
	shapeTransform->setScale(osg::Vec3(x,y,z));

	BROADCAST(this, "sfff", "setScale", x, y, z);
}

// ===================================================================
void ShapeNode::drawShape()
{

    pthread_mutex_lock(&pthreadLock);

	// remove the old shape:
	if (shapeTransform->containsNode(shapeGeode.get()))
	{
		shapeTransform->removeChild(shapeGeode.get());
		shapeGeode = NULL;
	}


	// TODO: this should only be added if this application is a graphical renderer.
	// There is no point to actually add the memory of the ShapeDrawable for apps
	// that do not need to use it!
	if (shape)
	{

		osg::TessellationHints* hints = new osg::TessellationHints;
		hints->setDetailRatio(GENERIC_SHAPE_RESOLUTION);

		shapeGeode = new osg::Geode();



		if (shape==PLANE) // OSG doesn't support planes yet (?!)
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
		shapeTransform->addChild(shapeGeode.get());
		shapeGeode->setName(string(id->s_name) + ".shapeGeode");
		optimizer.optimize(shapeGeode.get()); // ?
	}

	pthread_mutex_unlock(&pthreadLock);

	
	drawTexture();
	
}


// ===================================================================
void ShapeNode::drawTexture()
{

	if (!sceneManager->isGraphical()) return;
	
	
	//std::cout << "debug texturepath: " << texturePath <<  std::endl;


	if (texturePath==string("NULL"))
	{
		// remove current texture
		//shapeGeode->setStateSet( new osg::StateSet() );
	}

	else if (shapeGeode.valid())
	{

		//osg::ref_ptr<osg::Image> image;
		textureImage = osgDB::readImageFile( getAbsolutePath(texturePath).c_str() );
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

			osg::StateSet *shapeStateSet = new osg::StateSet();
			//osg::StateSet *shapeStateSet = shapeGeode->getOrCreateStateSet();

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

			shapeGeode->setStateSet( shapeStateSet );


		} else {
			std::cout << "ERROR (setTexture): The file " << texturePath << " is not a valid texture." << std::endl;
		}


	}

}

std::vector<lo_message> ShapeNode::getState ()
{
	// inherit state from base class
	std::vector<lo_message> ret = ReferencedNode::getState();

	lo_message msg;
	osg::Vec3 v;
	osg::Vec4 v4;

	//std::cout << "in getState. shape=" << shape << ", textureName=" << textureName << std::endl;


	msg = lo_message_new();
	v = this->getTranslation();
	lo_message_add(msg, "sfff", "setTranslation", v.x(), v.y(), v.z());
	ret.push_back(msg);

	msg = lo_message_new();
	v = this->getOrientation();
	lo_message_add(msg, "sfff", "setOrientation", v.x(), v.y(), v.z());
	ret.push_back(msg);

	msg = lo_message_new();
	v = this->getScale();
	lo_message_add(msg, "sfff", "setScale", v.x(), v.y(), v.z());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "si", "setShape", getShape());
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


	return ret;
}