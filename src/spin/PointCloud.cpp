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

#include <osg/Geode>
#include <osg/BlendColor>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osgUtil/Optimizer>
#include <osg/Geometry>
#include <osgUtil/SmoothingVisitor>
#include <osgSim/LightPointNode>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "PointCloud.h"
#include "SceneManager.h"
#include "spinApp.h"
#include "spinBaseContext.h"
#include "osgUtil.h"


//using namespace std;

namespace spin
{

// -----------------------------------------------------------------------------
// constructor:
PointCloud::PointCloud (SceneManager *sceneManager, char *initID) : GroupNode(sceneManager, initID)
{
	this->setName(std::string(id->s_name) + ".PointCloud");
	nodeType = "PointCloud";

	drawMode_ = LIGHTPOINTS;
    
    path_ = "NULL";
    customNode_ = gensym("NULL");
    
	color_ = osg::Vec4(1.0,1.0,0.0,1.0);
    spacing_ = 10.0;
    randomCoeff_ = 0.0;
    pointSize_ = 1.0;
    
    vertices_ = new osg::Vec3Array();
    colors_ = new osg::Vec4Array();
}

// destructor
PointCloud::~PointCloud()
{
    // TODO
}
// -----------------------------------------------------------------------------

void PointCloud::debug()
{
    GroupNode::debug();

    std::cout << "   numPoints: " << vertices_->size() << std::endl;

}

// -----------------------------------------------------------------------------
void PointCloud::callbackUpdate()
{
    GroupNode::callbackUpdate();
    
    if (redrawFlag_)
    {
        this->draw();
        redrawFlag_ = false;
    }
}

// -----------------------------------------------------------------------------

void PointCloud::loadFile(const char* filename)
{
    //path_ = getRelativePath(std::string(filename));
    path_ = filename;
    vertices_->clear();
    colors_->clear();
    //cloud_.clear();
    
    
    if (!sceneManager->isGraphical())
    {
        BROADCAST(this, "ss", "loadFile", filename);
        return;
    }
    
    if (path_ == "NULL") return;
    
    
    // load the file:
    bool success = false;
    pcl::PointCloud<pcl::PointXYZRGB> cloudXYZRGB;
    pcl::PointCloud<pcl::PointXYZ> cloudXYZ;

    try
    {
        //if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (path_, cloud_) == -1)  
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(path_, cloudXYZRGB) != -1)  
        {            
            std::cout << cloudXYZRGB.points.size() << " PointXYZRGB " << std::endl;
            for (int i=0; i<cloudXYZRGB.points.size(); i++)
            {
                vertices_->push_back (osg::Vec3 (cloudXYZRGB.points[i].x, cloudXYZRGB.points[i].y, cloudXYZRGB.points[i].z));
                uint32_t rgb_val_;
                memcpy(&rgb_val_, &(cloudXYZRGB.points[i].rgb), sizeof(uint32_t));
                
                uint32_t red,green,blue;
                blue=rgb_val_ & 0x000000ff;
                rgb_val_ = rgb_val_ >> 8;
                green=rgb_val_ & 0x000000ff;
                rgb_val_ = rgb_val_ >> 8;
                red=rgb_val_ & 0x000000ff;
                
                colors_->push_back (osg::Vec4f ((float)red/255.0f, (float)green/255.0f, (float)blue/255.0f,1.0f));
                
                success = true;
            }
        }
    }
    catch (pcl::InvalidConversionException e)
    {
        std::cout << "[PointCloud]: InvalidConversionException" << std::endl;
    }

    if (!success)
    try
    {
        if (pcl::io::loadPCDFile(path_, cloudXYZ) != -1)  
        {              
            for (int i=0; i<cloudXYZ.points.size(); i++)
            {
                vertices_->push_back (osg::Vec3 (cloudXYZ.points[i].x, cloudXYZ.points[i].y, cloudXYZ.points[i].z));
                colors_->push_back(color_);
            }
        
            success = true;
        }
    }
    catch (pcl::InvalidConversionException e)
    {
        std::cout << "[PointCloud]: InvalidConversionException" << std::endl;
    }
    
    if (success)
    {
        std::cout << "[PointCloud]: Loaded " << vertices_->size() << " data points from " << filename << std::endl;
        redrawFlag_ = true;
    } else
    {
        std::cout << "[PointCloud]: Couldn't read file " << path_ << std::endl;
    }
}        
        
void PointCloud::draw()
{
    if (!sceneManager->isGraphical()) return;

    // remove previous pointcloud:
    if (cloudGroup_.valid() && getAttachmentNode()->containsNode(cloudGroup_.get()))
    {
        getAttachmentNode()->removeChild(cloudGroup_.get());
        cloudGroup_ = NULL;
    }

    cloudGroup_ = new osg::PositionAttitudeTransform();
    cloudGroup_->setAttitude(osg::Quat(-osg::PI_2, osg::X_AXIS));
    cloudGroup_->setName(std::string(id->s_name) + ".CloudGroup");

    osg::TessellationHints* hints = new osg::TessellationHints;
    hints->setDetailRatio(1);
    hints->setCreateBackFace(false);

    if ((drawMode_>POINTS) && (drawMode_<=POLYGON))
    {
        osg::Geode* geode = new osg::Geode();
		osg::Geometry* geom = new osg::Geometry();
        geom->setVertexArray(vertices_);
        
        switch (drawMode_)
        {
            case LINES:
                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,vertices_->size()));
                break;
            case LINE_STRIP:
                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP,0,vertices_->size()));
                break;
            case LINE_LOOP:
                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP,0,vertices_->size()));
                break;
            case TRIANGLES:
                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES,0,vertices_->size()));
                break;
            case TRIANGLE_STRIP:
                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLE_STRIP,0,vertices_->size()));
                break;
            case TRIANGLE_FAN:
                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLE_FAN,0,vertices_->size()));
                break;
            case QUADS:
                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,vertices_->size()));
                break;
            case QUAD_STRIP:
                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUAD_STRIP,0,vertices_->size()));
                break;
            case POLYGON:
                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POLYGON,0,vertices_->size()));
                break;
            default: // POINTS
                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,vertices_->size()));
                break;
        }
        
        geode->setName(std::string(id->s_name) + ".CloudGeode");
        geode->addDrawable( geom );
        geom->setColorArray(colors_);
        geom->setColorBinding(osg::Geometry::BIND_OVERALL);
        
        osg::StateSet *ss = geode->getOrCreateStateSet();
		ss->setMode( GL_LIGHTING, osg::StateAttribute::OFF);
        
        cloudGroup_->addChild(geode);
    }
    
    else if (drawMode_ == LIGHTPOINTS)
    {
        osgSim::LightPointNode* lpn = new osgSim::LightPointNode;
        lpn->getLightPointList().reserve(vertices_->size());
        
        for (unsigned int i=0; i<vertices_->size(); i++)
        {
            osgSim::LightPoint lp;
            lp._color = colors_->at(i);
            lp._position = vertices_->at(i);
            lp._radius = pointSize_/1000.0;
            lpn->addLightPoint(lp);
        }
        cloudGroup_->addChild(lpn);
    }
        
    else if (drawMode_ == BOXES)
    {
        osg::Geode* geode = new osg::Geode();
        
        osg::Box* box = new osg::Box(osg::Vec3(0,0,0), pointSize_/500.0);
        osg::ShapeDrawable* boxDrawable = new osg::ShapeDrawable(box,hints);
        //boxDrawable->setColor(colors_->at(i));
        geode->addDrawable(boxDrawable);
        
        for (unsigned int i=0; i<vertices_->size(); i++)
        {
            osg::MatrixTransform *xform = new osg::MatrixTransform();
            
            /*
            // doesn't work:
            osg::BlendColor* bc = new osg::BlendColor(colors_->at(i));
            xform->getOrCreateStateSet()->setAttributeAndModes(bc, osg::StateAttribute::ON); 
            xform->getOrCreateStateSet()->setMode(osg::StateAttribute::BLENDCOLOR, osg::StateAttribute::ON); 
            */
            
            //xform->setMatrix(osg::Matrix::scale(osg::Vec3(pointSize_,pointSize_,pointSize_))*osg::Matrix::translate(vertices_->at(i)));
            xform->setMatrix(osg::Matrix::translate(vertices_->at(i)));
            xform->addChild(geode);
            cloudGroup_->addChild(xform);
        }
        /*
        for (unsigned int i=vertices_->size(); i<noStepsX*noStepsY; i++)
        {
            osg::MatrixTransform *xform = new osg::MatrixTransform();
            xform->setMatrix(osg::Matrix::scale(osg::Vec3(0,0,0)));
            xform->addChild(geode);
            cloudGroup_->addChild(xform);
        }
        */
            
        /*
        for (unsigned int i=0; i<vertexArray->size(); i++)
        {
            osg::Box* box = new osg::Box(vertexArray->at(i), pointSize_);
            osg::ShapeDrawable* boxDrawable = new osg::ShapeDrawable(box,hints);
            boxDrawable->setColor(colorArray->at(i));
            geode->addDrawable(boxDrawable);
        }
        cloudGroup->addChild(geode);
         */
    
        // TODO: SmoothingVisitor? Optimizer?
    }
    
    else if (drawMode_ == CUSTOM)
    {
        ReferencedNode *n = dynamic_cast<ReferencedNode*>(customNode_->s_thing);

        if (n)
        {
            for (unsigned int i=0; i<vertices_->size(); i++)
            {
                osg::MatrixTransform *xform = new osg::MatrixTransform();
                
                float scale = pointSize_/200.0;
                xform->setMatrix(osg::Matrix::scale(osg::Vec3(scale,scale,scale))*osg::Matrix::translate(vertices_->at(i)));
                xform->addChild(n);
                cloudGroup_->addChild(xform);
            }
        }
    }
    
    this->getAttachmentNode()->addChild(cloudGroup_.get());
}


// -----------------------------------------------------------------------------

void PointCloud::setCustomNode(const char* nodeID)
{
    
    ReferencedNode *n = sceneManager->getNode(nodeID);
    if (n)
    {
        customNode_ = gensym(nodeID);
        BROADCAST(this, "ss", "setCustomNode", nodeID);
        
        redrawFlag_ = true;
    }
    else
    {
        std::cout << "[PointCloud]: Could not set custom node. " << nodeID << " does not exist." << std::endl;
    }
    
}
const char* PointCloud::getCustomNode() const
{
    ReferencedNode *n = dynamic_cast<ReferencedNode*>(customNode_->s_thing);
    if (n) return customNode_->s_name;
    return "NULL"; 
}


void PointCloud::setDrawMode (DrawMode mode)
{
	if (this->drawMode_ != (int)mode)
	{
		this->drawMode_ = mode;
		BROADCAST(this, "si", "setDrawMode", getDrawMode());
        
        redrawFlag_ = true;
	}
}

void PointCloud::setSpacing (float spacing)
{
	if (this->spacing_ != spacing)
	{
		this->spacing_ = spacing;
		BROADCAST(this, "sf", "setSpacing", this->spacing_);
        redrawFlag_ = true;

	}
}

void PointCloud::setRandomCoeff (float randomCoeff)
{
	if (this->randomCoeff_ != randomCoeff)
	{
		this->randomCoeff_ = randomCoeff;
		BROADCAST(this, "sf", "setRandomCoeff", this->randomCoeff_);
        redrawFlag_ = true;

	}
}
    
void PointCloud::setPointSize (float size)
{
    if (this->pointSize_ != size)
    {
        this->pointSize_ = size;
        BROADCAST(this, "sf", "setPointSize", this->pointSize_);
        redrawFlag_ = true;

    }
}

void PointCloud::setColor (float r, float g, float b, float a)
{
	if (this->color_ != osg::Vec4(r,g,b,a))
	{
		this->color_ = osg::Vec4(r,g,b,a);
		BROADCAST(this, "sffff", "setColor", r, g, b, a);
        redrawFlag_ = true;

	}
}

// -----------------------------------------------------------------------------
std::vector<lo_message> PointCloud::getState () const
{
    // inherit state from base class
	std::vector<lo_message> ret = GroupNode::getState();

    lo_message msg;

    msg = lo_message_new();
    lo_message_add(msg, "ss", "setCustomNode", getCustomNode());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "si", "setDrawMode", getDrawMode());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "sf", "setSpacing", getSpacing());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "sf", "setRandomCoeff", getRandomCoeff());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "sf", "setPointSize", getPointSize());
    ret.push_back(msg);

    msg = lo_message_new();
    osg::Vec4 v = this->getColor();
    lo_message_add(msg, "sffff", "setColor", v.x(), v.y(), v.z(), v.w());
    ret.push_back(msg);
    
    msg = lo_message_new();
    lo_message_add(msg, "ss", "loadFile", path_.c_str());
    ret.push_back(msg);


    return ret;
}

} // end of namespace spin

