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

#include "spinApp.h"
#include "spinBaseContext.h"
#include "osgUtil.h"
#include "GeometryNode.h"
#include "SceneManager.h"


#include "ImageTexture.h"
#include "VideoTexture.h"
#include "SharedVideoTexture.h"

using namespace std;

extern pthread_mutex_t sceneMutex;

namespace spin
{

// ===================================================================
// constructor:
GeometryNode::GeometryNode (SceneManager *sceneManager, char *initID) : GroupNode(sceneManager, initID)
{
	this->setName(string(id->s_name) + ".GeometryNode");
	nodeType = "GeometryNode";

    updateFlag_ = false;

    numVertices_ = 4;
    osg::Vec3Array *vertexArray_ = new osg::Vec3Array(numVertices_);
    osg::Vec4Array *colorArray_ = new osg::Vec4Array(numVertices_);
    osg::Vec2Array *texcoordArray_ = new osg::Vec2Array(numVertices_);
    
    // initialize the arrays:
    
    //vertexArray_->assign(numVertices_, osg::Vec3(0,0,0));
    (*vertexArray_)[0].set(-0.5f, 0.0f,  0.5f);
    (*vertexArray_)[1].set(-0.5f, 0.0f, -0.5f);
    (*vertexArray_)[2].set( 0.5f, 0.0f, -0.5f);
    (*vertexArray_)[3].set( 0.5f, 0.0f,  0.5f);

    colorArray_->assign(numVertices_, osg::Vec4(1,1,1,1));
    
    //texcoordArray_->assign(numVertices_, osg::Vec2(0,0));
	(*texcoordArray_)[0].set(0.0f,1.0f);
	(*texcoordArray_)[1].set(0.0f,0.0f);
	(*texcoordArray_)[2].set(1.0f,0.0f);
	(*texcoordArray_)[3].set(1.0f,1.0f);

    
    geode_ = new osg::Geode();
    this->getAttachmentNode()->addChild(geode_.get());
    geode_->setName(string(id->s_name) + ".geode");

    geometry_ = new osg::Geometry();
    geometry_->setVertexArray(vertexArray_);
    
    /*
     // set the normal in the same way color.
     osg::Vec3Array* normals = new osg::Vec3Array;
     normals->push_back(osg::Vec3(0.0f,-1.0f,0.0f));
     geom->setNormalArray(normals);
     geom->setNormalBinding(osg::Geometry::BIND_OVERALL);
     */
    
	geometry_->setColorArray(colorArray_);
	geometry_->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    
	geometry_->setTexCoordArray(0,texcoordArray_);
    
	geometry_->addPrimitiveSet(new osg::DrawArrays(GL_QUADS,0,numVertices_));
	
    geode_->addDrawable(geometry_);
}

// -----------------------------------------------------------------------------
// destructor
GeometryNode::~GeometryNode()
{
}

    
    
// -----------------------------------------------------------------------------
void GeometryNode::callbackUpdate()
{
    GroupNode::callbackUpdate();
    
    if (updateFlag_)
    {
        geometry_->dirtyDisplayList();
        updateFlag_ = false;
    }
}
    
// -----------------------------------------------------------------------------

void GeometryNode::updateStateSet()
{
	osg::ref_ptr<ReferencedStateSet> ss = dynamic_cast<ReferencedStateSet*>(stateset_->s_thing);
	if (geode_.valid() && ss.valid()) geode_->setStateSet( ss.get() );
}

void GeometryNode::setNumVertices(int i)
{
    if (numVertices_ != i)
    {
        numVertices_ = i;
        
        osg::Vec3Array *vertexArray = dynamic_cast<osg::Vec3Array*>(geometry_->getVertexArray());
        vertexArray->resize(i, osg::Vec3(0,0,0));
        
        osg::Vec4Array *colorArray = dynamic_cast<osg::Vec4Array*>(geometry_->getColorArray());
        colorArray->resize(i, osg::Vec4(1,1,1,1));
        
        osg::Vec2Array *texcoordArray = dynamic_cast<osg::Vec2Array*>(geometry_->getTexCoordArray(0));
        texcoordArray->resize(i, osg::Vec2(0,0));
        
        
        if (numVertices_ < 3)
        {
            geometry_->setPrimitiveSet(0, new osg::DrawArrays(GL_POINTS,0,numVertices_));
        }
        else if (numVertices_ % 3 == 0)
        {
            geometry_->setPrimitiveSet(0, new osg::DrawArrays(GL_TRIANGLES,0,numVertices_));
        }
        else if (numVertices_ % 4 == 0)
        {
            geometry_->setPrimitiveSet(0, new osg::DrawArrays(GL_QUADS,0,numVertices_));
        }
        else 
        {
            geometry_->setPrimitiveSet(0, new osg::DrawArrays(GL_LINES,0,numVertices_));
        }
        
        updateFlag_ = true;
        
    }
    
    BROADCAST(this, "si", "setNumVertices", numVertices_);
}
void GeometryNode::setVertex(int index, float x, float y, float z)
{
    if ((index>=0) && (index<numVertices_))
    {
        osg::Vec3Array *a = dynamic_cast<osg::Vec3Array*>(geometry_->getVertexArray());
        a->at(index)[0] = x;
        a->at(index)[1] = y;
        a->at(index)[2] = z;
        updateFlag_ = true;
        BROADCAST(this, "sifff", "setVertex", index, x, y, z);
    }
    else
    {
        std::cout << "GeometryNode: index " << index << " is greater than the maximum number of vertices (" << numVertices_ << ")" << std::endl;
    }
}
void GeometryNode::setColor(int index, float red, float green, float blue, float alpha)
{
    if ((index>=0) && (index<numVertices_))
    {
        osg::Vec4Array *a = dynamic_cast<osg::Vec4Array*>(geometry_->getColorArray());
        a->at(index)[0] = red;
        a->at(index)[1] = green;
        a->at(index)[2] = blue;
        a->at(index)[3] = alpha;
        updateFlag_ = true;
        BROADCAST(this, "siffff", "setColor", index, red, green, blue, alpha);
    }
    else
    {
        std::cout << "GeometryNode: index " << index << " is greater than the maximum number of vertices (" << numVertices_ << ")" << std::endl;
    }   
}
    
void GeometryNode::setTexCoord(int index, float x, float y)
{
    if ((index>=0) && (index<numVertices_))
    {
        osg::Vec2Array *a = dynamic_cast<osg::Vec2Array*>(geometry_->getTexCoordArray(0));
        a->at(index)[0] = x;
        a->at(index)[1] = y;
        updateFlag_ = true;
        BROADCAST(this, "siff", "setTexCoord", index, x, y);
    }
    else
    {
        std::cout << "GeometryNode: index " << index << " is greater than the maximum number of vertices (" << numVertices_ << ")" << std::endl;
    }   
}

// -----------------------------------------------------------------------------

std::vector<lo_message> GeometryNode::getState () const
{
	// inherit state from base class
	std::vector<lo_message> ret = GroupNode::getState();

	lo_message msg;
	osg::Vec3 v3;
	osg::Vec4 v4;

	// put this one last:
	/*
	msg = lo_message_new();
	lo_message_add(msg, "sfffffffffff", "setVertices", getVertices());
	ret.push_back(msg);
    */
    
    osg::Vec3Array *vArray = dynamic_cast<osg::Vec3Array*>(geometry_->getVertexArray());
    for (unsigned int i=0; i<vArray->getNumElements(); i++)
    {
        //v3 = (*vArray)[i];
        
        msg = lo_message_new();
        lo_message_add(msg, "sifff", "setVertex", i, (*vArray)[i].x(), (*vArray)[i].y(), (*vArray)[i].z());
        ret.push_back(msg);
    }
    
    osg::Vec4Array *cArray = dynamic_cast<osg::Vec4Array*>(geometry_->getColorArray());
    for (unsigned int i=0; i<cArray->getNumElements(); i++)
    {
        msg = lo_message_new();
        lo_message_add(msg, "siffff", "setColor", i, (*cArray)[i].x(), (*cArray)[i].y(), (*cArray)[i].z(), (*cArray)[i].w());
        ret.push_back(msg);
    }
    
    
    osg::Vec2Array *tArray = dynamic_cast<osg::Vec2Array*>(geometry_->getTexCoordArray(0));
    for (unsigned int i=0; i<tArray->getNumElements(); i++)
    {
        msg = lo_message_new();
        lo_message_add(msg, "siff", "setTexCoord", i, (*tArray)[i].x(), (*tArray)[i].y());
        ret.push_back(msg);
    }

	return ret;
}

} // end of namespace spin

