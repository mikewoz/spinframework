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

#include "gridnode.h"
#include "scenemanager.h"
#include "spinapp.h"
#include "spinbasecontext.h"
#include "osgutil.h"

extern pthread_mutex_t sceneMutex;

namespace spin
{

// *****************************************************************************
// constructor:
GridNode::GridNode (SceneManager *sceneManager, const char* initID) : ReferencedNode(sceneManager, initID)
{
	this->setName(this->getID() + ".GridNode");
	this->setNodeType("GridNode");

	_size = 10;
	_color = osg::Vec4(0.5,0.5,0.5,1);

	this->setNodeMask(DEBUGVIEW_NODE_MASK); // nodemask info in spinUtil.h

	drawGrid();
}

// *****************************************************************************
// destructor
GridNode::~GridNode()
{

}


// *****************************************************************************
void GridNode::setSize (int s)
{
	if (this->_size != s)
	{
		this->_size = s;
		drawGrid();
		BROADCAST(this, "si", "setSize", (int) this->_size);
	}
}

void GridNode::setColor (float r, float g, float b, float a)
{
	if (this->_color != osg::Vec4(r,g,b,a))
	{
		this->_color = osg::Vec4(r,g,b,a);
		drawGrid();
		BROADCAST(this, "sffff", "setColor", r, g, b, a);
	}
}

// *****************************************************************************

void GridNode::drawGrid()
{
	if (!sceneManager_->isGraphical()) return;
	
    pthread_mutex_lock(&sceneMutex);

	// remove the old ray:
	if (this->getAttachmentNode()->containsNode(gridGeode.get()))
	{
		this->getAttachmentNode()->removeChild(gridGeode.get());
		gridGeode = NULL;
	}

	if (this->_size)
	{
		int i,j;
		int numVertices = 8 + (_size*8);
		
		
		gridGeode = new osg::Geode();

		osg::Geometry* gridLines = new osg::Geometry();
		
		osg::Vec3Array* normals = new osg::Vec3Array;
		osg::Vec3* myCoords = new osg::Vec3[numVertices];
		osg::Vec4Array* colors = new osg::Vec4Array;
		
		for (i = 0; i <= _size; i++)
		{
			myCoords[i+0+(i*7)] = osg::Vec3(-_size,  i, 0.0);
			myCoords[i+1+(i*7)] = osg::Vec3( _size,  i, 0.0);
			myCoords[i+2+(i*7)] = osg::Vec3(-_size, -i, 0.0);
			myCoords[i+3+(i*7)] = osg::Vec3( _size, -i, 0.0);

			myCoords[i+4+(i*7)] = osg::Vec3( i, -_size, 0.0);
			myCoords[i+5+(i*7)] = osg::Vec3( i,  _size, 0.0);
			myCoords[i+6+(i*7)] = osg::Vec3(-i, -_size, 0.0);
			myCoords[i+7+(i*7)] = osg::Vec3(-i,  _size, 0.0);

			for (j=0; j<8; j++)
			{
				if (i==0) colors->push_back(osg::Vec4(1.0, 1.0, 1.0, 0.0));
				else colors->push_back(_color);
				normals->push_back(osg::Vec3(0.0, 0.0, 1.0));
			}
		}

		//for (i = 0; i < numVertices; i++)
		//	std::cout << i << ": (" << myCoords[i].x() << "," << myCoords[i].y() << "," << myCoords[i].z() << ")" << std::endl;
		
		
		osg::Vec3Array* vertices = new osg::Vec3Array(numVertices,myCoords);
		
		// pass the created vertex array to the points geometry object.
		gridLines->setVertexArray(vertices);
		
		gridLines->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, numVertices));
		
		// pass the color arry to points geometry, note the binding to tell the geometry
		// that only use one color for the whole object.
		gridLines->setColorArray(colors);
		//gridLines->setColorBinding(osg::Geometry::BIND_OVERALL);
		
		// create the normals
		//gridLines->setNormalArray(normals);
		//gridLines->setNormalBinding(osg::Geometry::BIND_OVERALL);

        
#ifdef OSG_MIN_VERSION_REQUIRED
#if OSG_MIN_VERSION_REQUIRED(3,0,0)
        osgUtil::SmoothingVisitor::smooth(*gridLines, osg::PI);
#else
        osgUtil::SmoothingVisitor::smooth(*gridLines);
#endif
#else
        osgUtil::SmoothingVisitor::smooth(*gridLines);
#endif
        
        
		gridGeode->addDrawable(gridLines);
		
		osgUtil::Optimizer optimizer;
		optimizer.optimize(gridGeode.get());

		osg::StateSet* stateset = new osg::StateSet;
		stateset->setMode( GL_LIGHTING, osg::StateAttribute::OFF ); //(GL_LIGHTING, osg::StateAttribute::PROTECTED | osg::StateAttribute::OFF)
		gridGeode->setStateSet( stateset );
		
		gridGeode->setName("grid.geode");
		
		this->getAttachmentNode()->addChild(gridGeode.get());

	}
		
    pthread_mutex_unlock(&sceneMutex);

}


// *****************************************************************************

std::vector<lo_message> GridNode::getState () const
{
	// inherit state from base class
	std::vector<lo_message> ret = ReferencedNode::getState();

	lo_message msg;

	msg = lo_message_new();
	lo_message_add(msg, "si", "setSize", getSize());
	ret.push_back(msg);

	msg = lo_message_new();
	osg::Vec4 v = this->getColor();
	lo_message_add(msg, "sffff", "setColor", v.x(), v.y(), v.z(), v.w());
	ret.push_back(msg);

	return ret;
}

} // end of namespace spin

