#include <osg/ShapeDrawable>
#include <osgUtil/Optimizer>
#include <osg/Geometry>
#include <osgUtil/SmoothingVisitor>

#include "asRay.h"
#include "asSceneManager.h"

using namespace std;

//extern asSceneManager *sceneManager;



// *****************************************************************************
// constructor:
asRay::asRay (asSceneManager *sceneManager, char *initID) : asReferenced(sceneManager, initID)
{
	this->setName(string(id->s_name) + ".asRay");
	nodeType = "asRay";

	visible = true;
	length = 100;
	thickness = 0.25;
	color = osg::Vec4(1,1,1,1);

	
	this->setNodeMask(DEBUGVIEW_NODE_MASK); // nodemask info in asGlobals.h
	
	drawRay();
}

// *****************************************************************************
// destructor
asRay::~asRay()
{

}


// *****************************************************************************
void asRay::setVisible (int b)
{
	if (this->visible != (bool)b)
	{
		this->visible = (bool) b;
		drawRay();
		BROADCAST(this, "si", "setVisible", (int) this->visible);	
	}
}

void asRay::setLength (float len)
{
	if (this->length != len)
	{
		this->length = len;	
		drawRay();
		BROADCAST(this, "sf", "setLength", this->length);
	}
}

void asRay::setThickness (float thk)
{
	if (this->thickness != thk)
	{
		this->thickness = thk;
		drawRay();	
		BROADCAST(this, "sf", "setThickness", this->thickness);
	}
}

void asRay::setColor (float r, float g, float b, float a)
{
	if (this->color != osg::Vec4(r,g,b,a))
	{
		this->color = osg::Vec4(r,g,b,a);
		drawRay();
		BROADCAST(this, "sffff", "setColor", r, g, b, a);
	}
}

// *****************************************************************************

void asRay::drawRay()
{

	// remove the old ray:
	if (this->containsNode(rayGeode.get()))
	{
		this->removeChild(rayGeode.get());
		rayGeode = NULL;
	}
		
	if (this->visible)
	{
		rayGeode = new osg::Geode();
		
		if (0) 
		{
			// the old cylinder method:
			
			osg::TessellationHints* hints = new osg::TessellationHints;
			hints->setDetailRatio(GENERIC_SHAPE_RESOLUTION);
				
			// osg::Cylinder(center, radius, height)
			osg::Cylinder* cylndr = new osg::Cylinder(osg::Vec3(0.0f,this->length/2,0.0f), this->thickness, this->length);
			cylndr->setRotation(osg::Quat(osg::PI/2,osg::Vec3(1,0,0)));
			
			osg::ShapeDrawable* rayDrawable = new osg::ShapeDrawable(cylndr);
			rayDrawable->setColor(this->color);
			
			rayGeode->addDrawable(rayDrawable);
		
		} else {
			
			// just draw a line
			osg::Geometry* rayGeom = new osg::Geometry();
			
			osg::Vec3Array* vertices = new osg::Vec3Array;
			vertices->push_back(osg::Vec3(0,0,0));
			vertices->push_back(osg::Vec3(0,this->length,0));
			
			osg::Vec4Array* vertColors = new osg::Vec4Array;
			vertColors->push_back(this->color);
			rayGeom->setColorArray(vertColors);
			rayGeom->setColorBinding(osg::Geometry::BIND_OVERALL);
			
			rayGeom->setVertexArray(vertices);
			rayGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, vertices->size()));

			osgUtil::SmoothingVisitor::smooth(*rayGeom);
				
			rayGeode->addDrawable(rayGeom);
		
		}
		

		


		if (rayGeode.valid())
		{
			this->addChild(rayGeode.get());
			rayGeode->setName(string(id->s_name) + ".rayGeode");
			osgUtil::Optimizer optimizer;
			optimizer.optimize(rayGeode.get());
		}
	}	

}


// *****************************************************************************

std::vector<lo_message> asRay::getState ()
{
	// inherit state from base class
	std::vector<lo_message> ret = asReferenced::getState();
	
	lo_message msg;
	
	msg = lo_message_new();
	lo_message_add(msg, "si", "setVisible", (int)visible);
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "sf", "setLength", length);
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "sf", "setThickness", thickness);
	ret.push_back(msg);

	msg = lo_message_new();
	osg::Vec4 v = this->getColor();
	lo_message_add(msg, "sffff", "setColor", v.x(), v.y(), v.z(), v.w());
	ret.push_back(msg);
	
	return ret;
}