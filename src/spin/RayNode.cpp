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

#include <osg/ShapeDrawable>
#include <osgUtil/Optimizer>
#include <osg/Geometry>
#include <osgUtil/SmoothingVisitor>

#include "RayNode.h"
#include "SceneManager.h"
#include "spinApp.h"
#include "spinBaseContext.h"
#include "osgUtil.h"

using namespace std;

//extern SceneManager *sceneManager;



// *****************************************************************************
// constructor:
RayNode::RayNode (SceneManager *sceneManager, char *initID) : GroupNode(sceneManager, initID)
{
	this->setName(string(id->s_name) + ".RayNode");
	nodeType = "RayNode";

	visible = true;
	length = 100;
	thickness = 0.01;
	color = osg::Vec4(1.0,1.0,0.0,1.0);


	this->setNodeMask(DEBUGVIEW_NODE_MASK); // nodemask info in spinUtil.h

	drawRay();
}

// *****************************************************************************
// destructor
RayNode::~RayNode()
{

}


// *****************************************************************************
void RayNode::setVisible (int b)
{
	if (this->visible != (bool)b)
	{
		this->visible = (bool) b;
		drawRay();
		BROADCAST(this, "si", "setVisible", (int) this->visible);
	}
}

void RayNode::setLength (float len)
{
	if (this->length != len)
	{
		this->length = len;
		drawRay();
		BROADCAST(this, "sf", "setLength", this->length);
	}
}

void RayNode::setThickness (float thk)
{
	if (this->thickness != thk)
	{
		this->thickness = thk;
		drawRay();
		BROADCAST(this, "sf", "setThickness", this->thickness);
	}
}

void RayNode::setColor (float r, float g, float b, float a)
{
	if (this->color != osg::Vec4(r,g,b,a))
	{
		this->color = osg::Vec4(r,g,b,a);
		drawRay();
		BROADCAST(this, "sffff", "setColor", r, g, b, a);
	}
}

// *****************************************************************************

void RayNode::drawRay()
{

	// remove the old ray:
	if (getAttachmentNode()->containsNode(rayGeode.get()))
	{
		getAttachmentNode()->removeChild(rayGeode.get());
		rayGeode = NULL;
	}

	if (this->visible)
	{
        rayGeode = new osg::Geode();

        if (thickness <= 0.01)
        {
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
        else
        {
        	// draw a cylinder
        	osg::Cylinder *cyl;
        	osg::ShapeDrawable* cylDrawable;
        	cyl = new osg::Cylinder(osg::Vec3(0,length/2,0), thickness, length);
        	cyl->setRotation(osg::Quat(-osg::PI/2, osg::X_AXIS));

        	cylDrawable = new osg::ShapeDrawable(cyl);
        	cylDrawable->setColor(color);
        	rayGeode->addDrawable(cylDrawable);
        }

        if (rayGeode.valid())
        {
        	getAttachmentNode()->addChild(rayGeode.get());
            rayGeode->setName(string(id->s_name) + ".rayGeode");
            osgUtil::Optimizer optimizer;
            optimizer.optimize(rayGeode.get());
        }
    }

}


// *****************************************************************************

std::vector<lo_message> RayNode::getState ()
{
    // inherit state from base class
	std::vector<lo_message> ret = GroupNode::getState();

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
