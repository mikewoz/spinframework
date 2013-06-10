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

#include "spinapp.h"
#include "spinbasecontext.h"
#include "osgutil.h"
#include "shapenode.h"
#include "scenemanager.h"

#include "imagetexture.h"
#include "videotexture.h"
#include "sharedvideotexture.h"
#include "shaderutil.h"

extern pthread_mutex_t sceneMutex;

namespace spin
{

// ===================================================================
// constructor:
ShapeNode::ShapeNode (SceneManager *sceneManager, const char* initID) : GroupNode(sceneManager, initID)
{
	this->setName(this->getID() + ".ShapeNode");
	this->setNodeType("ShapeNode");

	_color = osg::Vec4(1.0,1.0,1.0,1.0);	

	shape = BOX;
	billboard = RELATIVE; // ie, no billboard
	texturePath = "NULL";
	renderBin = -1;
	lightingEnabled = -1;
    singleSided_ = false;
    detailRatio_ = 1.0;
    
    // quick shader test:
    if (0)
    {
        osg::Geode* geode = new osg::Geode();
        osg::ShapeDrawable* drawable = new osg::ShapeDrawable(new osg::Box(osg::Vec3(1,1,1), 1));
        geode->addDrawable(drawable);
        sceneManager_->worldNode->addChild(geode);

        osg::Shader* vshader = new osg::Shader(osg::Shader::VERTEX, microshaderVertSource );
        osg::Shader* fshader = new osg::Shader(osg::Shader::FRAGMENT, microshaderFragSource );
        osg::Program * prog = new osg::Program;
        prog->addShader ( vshader );
        prog->addShader ( fshader );
        geode->getOrCreateStateSet()->setAttributeAndModes( prog, osg::StateAttribute::ON );
    }
    // end shader test
    
    drawShape();
}

// ===================================================================
// destructor
ShapeNode::~ShapeNode()
{
	//if (sceneManager_->sharedStateManager.valid()) sceneManager_->sharedStateManager->prune();
}

// ===================================================================
// ======================== SET METHODS: =============================
// ===================================================================

void ShapeNode::setContext (const char *newvalue)
{
	if (this->getContextString() == std::string(newvalue)) return;
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

        if ( _shapeDrawable.get() ) {
            _shapeDrawable->setColor( _color );

        } else if ( shapeGeode->getDrawable(0)->asGeometry() ) {            
            osg::Vec4Array* c = dynamic_cast<osg::Vec4Array*>( shapeGeode->getDrawable(0)->asGeometry()->getColorArray() );
            if (c) {
                (*c)[0].set(r,g,b,a);  // might be pure evil.
                shapeGeode->getDrawable(0)->dirtyDisplayList();
            } else drawShape();
        }
        else drawShape();  
        
		BROADCAST(this, "sffff", "setColor", r, g, b, a);
	}
}

// ===================================================================

void ShapeNode::updateStateSet()
{
	osg::ref_ptr<ReferencedStateSet> ss = dynamic_cast<ReferencedStateSet*>(stateset_->s_thing);
	if (shapeGeode.valid() && ss.valid()) shapeGeode->setStateSet( ss.get() );
}

// ===================================================================
void ShapeNode::setRenderBin (int i)
{
    // This is a legacy method before we had referenced statesets
    // in SPIN. Thus, if the user tries to set the renderbin here
    // we will apply it to the stateset instead. And if there is
    // no stateset, then we try to apply it directly to the
    // shapeGeode

    osg::ref_ptr<ReferencedStateSet> ss = dynamic_cast<ReferencedStateSet*>(stateset_->s_thing);
    if (ss.valid() && (renderBin>0))
    {
        // no, it's not safe to change the stateset because it might
        // shared with other nodes, and this might cause undesired
        // effects
        //ss->setRenderBin(i);
        std::cout << "Cannot setRenderBin on '" << getID() << "' because that node has stateset '" << ss->getID() << "'. Change the RenderBin on the stateset instead." << std::endl;
    }
    else 
    {
        renderBin = i;
        
        if (shapeGeode.valid() && (renderBin>0))
        {
            osg::StateSet *ss = shapeGeode->getOrCreateStateSet();
            ss->setRenderBinDetails( (int)renderBin, "RenderBin");
        }
        
        BROADCAST(this, "si", "setRenderBin", renderBin);
    }
}

void ShapeNode::setLighting (int i)
{
    osg::ref_ptr<ReferencedStateSet> ss = dynamic_cast<ReferencedStateSet*>(stateset_->s_thing);
    if (ss.valid() && (lightingEnabled>=0))
    {
        //ss->setLighting(i);
        std::cout << "Cannot setLighting on '" << getID() << "' because that node has stateset '" << ss->getID() << "'. Enable/disable lighting on the stateset instead." << std::endl;
    }
    else
    {
        lightingEnabled = (bool)i;
        
        if (shapeGeode.valid() && (lightingEnabled>=0))
        {
            osg::StateSet *ss = shapeGeode->getOrCreateStateSet();
            if (lightingEnabled) ss->setMode( GL_LIGHTING, osg::StateAttribute::ON );
            else ss->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
        }
        BROADCAST(this, "si", "setLighting", getLighting());
    }
	
}

void ShapeNode::setSingleSided (int singleSided)
{
    singleSided_ = singleSided;
    
    if (shapeGeode.valid())
    {
        osg::StateSet *ss = shapeGeode->getOrCreateStateSet();
        if (singleSided_)
            ss->setMode( GL_CULL_FACE, osg::StateAttribute::ON );
        else
            ss->setMode( GL_CULL_FACE, osg::StateAttribute::OFF );
    }
    else
    {
        drawShape();
    }

    BROADCAST(this, "si", "setSingleSided", getSingleSided());
}

void ShapeNode::setDetailRatio (float detailRatio)
{
    detailRatio_ = detailRatio;
    drawShape();
    BROADCAST(this, "sf", "setDetailRatio", getDetailRatio());
}

// ===================================================================
void ShapeNode::drawShape()
{
    pthread_mutex_lock(&sceneMutex);

	// remove the old shape:
	if (this->getAttachmentNode()->containsNode(shapeGeode.get()))
	{
		this->getAttachmentNode()->removeChild(shapeGeode.get());
		shapeGeode = NULL;
	}

	// only draw shape if context matches host or context is empty
    // Have to use to a string "NULL" because OSC cannot do empty strings
	//bool drawOnThisHost = (not spinApp::Instance().getContext()->isServer()) 
    //        and (this->getContextString() == getHostname() or this->getContextString() == "NULL");

	bool drawOnThisHost = ((this->getContextString() == spinApp::Instance().getUserID()) or
	                       (this->getContextString() == "NULL"));

	
	// TODO: this should only be added if this application is a graphical renderer.
	// There is no point to actually add the memory of the ShapeDrawable for apps
	// that do not need to use it!
	// ACTUALLY, the server needs to know about the geometry to compute 
	// intersections, determine bounding regions, radius, etc...
	//if (shape && !ignoreOnThisHost)
	if (shape and drawOnThisHost)
	{
		osg::TessellationHints* hints = new osg::TessellationHints;
		hints->setDetailRatio(detailRatio_);

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
                default:
                    break;
			}
			shapeGeode = b;
			
		} else {
			shapeGeode = new osg::Geode();
		}

        //shapeGeode->removeDrawable( _shapeDrawable.release() );
        _shapeDrawable.release();

		if (shape==PLANE) // OSG doesn't support planes
        {
            shapeGeode->addDrawable(createPlane(AS_UNIT_SCALE * .5, _color));
			//if ( shapeGeode->getNumDrawables() == 0 ) shapeGeode->addDrawable(createPlane(AS_UNIT_SCALE * .5, _color));
            //else shapeGeode->setDrawable(0, createPlane(AS_UNIT_SCALE * .5, _color));
		}
        else if (shape==DISC)
        {
            shapeGeode->addDrawable(createCone(0, AS_UNIT_SCALE * .5, _color));
            //if ( shapeGeode->getNumDrawables() == 0 ) shapeGeode->addDrawable(createCone(0, AS_UNIT_SCALE * .5, _color));
            //else shapeGeode->setDrawable(0, createCone(0, AS_UNIT_SCALE * .5, _color));
        }
		else if (shape==SPHERE)
		{
			_shapeDrawable = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.0f,0.0f,0.0f),AS_UNIT_SCALE*.5), hints);
		}
		else if (shape==BOX)
		{
			_shapeDrawable = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0.0f,0.0f,0.0f), AS_UNIT_SCALE), hints);
		}
		else if (shape==CYLINDER)
		{
			_shapeDrawable = new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3(0.0f,0.0f,0.0f), AS_UNIT_SCALE*.25,AS_UNIT_SCALE), hints);
		}
		else if (shape==CAPSULE)
		{
			_shapeDrawable = new osg::ShapeDrawable(new osg::Capsule(osg::Vec3(0.0f,0.0f,0.0f), AS_UNIT_SCALE*.25,AS_UNIT_SCALE), hints);
		}
		else if (shape==CONE)
		{
			_shapeDrawable = new osg::ShapeDrawable(new osg::Cone(osg::Vec3(0.0f,0.0f,0.0f), AS_UNIT_SCALE*.25,AS_UNIT_SCALE), hints);
		}
		else {
			//return;
		}         
		

        if ( _shapeDrawable.get() ) {
            shapeGeode->addDrawable( _shapeDrawable );
			//if ( shapeGeode->getNumDrawables() == 0 ) shapeGeode->addDrawable( _shapeDrawable );
            //else shapeGeode->setDrawable( 0, _shapeDrawable );

			_shapeDrawable->setColor(_color);
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

		// if this shape has a stateset, then lighting is defined there.
		// Otherwise, we have an internal lighting parameter for the shape:
		if (!stateset_->s_thing)
		{
			if (lightingEnabled) ss->setMode( GL_LIGHTING, osg::StateAttribute::ON );
			else ss->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
		}
		
        if (singleSided_)
            ss->setMode( GL_CULL_FACE, osg::StateAttribute::ON );
        else
            ss->setMode( GL_CULL_FACE, osg::StateAttribute::OFF );
        
		this->getAttachmentNode()->addChild(shapeGeode.get());
		shapeGeode->setName(this->getID() + ".shapeGeode");
		optimizer.optimize(shapeGeode.get()); // ?
	}

	pthread_mutex_unlock(&sceneMutex);

	
	updateStateSet();

}

std::vector<lo_message> ShapeNode::getState () const
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

if (getRenderBin() > 0)
{
	msg = lo_message_new();
	lo_message_add(msg, "si", "setRenderBin", getRenderBin());
	ret.push_back(msg);
}

	msg = lo_message_new();
	lo_message_add(msg, "si", "setSingleSided", getSingleSided());
	ret.push_back(msg);

if (getLighting() >= 0)
{
	msg = lo_message_new();
	lo_message_add(msg, "si", "setLighting", getLighting());
	ret.push_back(msg);
}

	// put this one last:	
	msg = lo_message_new();
	lo_message_add(msg, "si", "setShape", getShape());
	ret.push_back(msg);

	return ret;
}

} // end of namespace spin

