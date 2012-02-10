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
#include <osg/ShapeDrawable>
#include <osgUtil/Optimizer>
#include <osg/Geometry>
#include <osgUtil/SmoothingVisitor>
#include <osgSim/LightPointNode>



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
    
    valid_ = false;
	color_ = osg::Vec4(1.0,1.0,0.0,1.0);
    spacing_ = 10.0;
    randomCoeff_ = 0.0;
    pointSize_ = 1.0;

    
    if (sceneManager->isGraphical())
    {
                
        valid_ = true;
    }

}

// destructor
PointCloud::~PointCloud()
{
    // TODO
}

// -----------------------------------------------------------------------------
void PointCloud::callbackUpdate()
{
    GroupNode::callbackUpdate();
    

}

// -----------------------------------------------------------------------------

void PointCloud::loadFile(const char* filename)
{
    //path_ = getRelativePath(std::string(filename));
    path_ = filename;
    
    
    if (!sceneManager->isGraphical())
    {
        BROADCAST(this, "ss", "loadFile", filename);
    }
    else if (path_ != "NULL")
    try
    {
        // load the file:
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (path_, cloud_) == -1)  
        {
            std::cout << "[PointCloud]: Couldn't read file " << path_ << std::endl;
        }
        else
        {
            std::cout << "[PointCloud]: Loaded " << cloud_.width * cloud_.height << " data points from " << filename << std::endl;
            
            
            if (getAttachmentNode()->containsNode(cloudGeode_.get()))
            {
                getAttachmentNode()->removeChild(cloudGeode_.get());
                cloudGeode_ = NULL;
            }
            
            
            cloudGeode_=osg::ref_ptr<osg::Geode>(new osg::Geode());
            osg::ref_ptr<osg::Geometry> geometry (new osg::Geometry());
            
            osg::ref_ptr<osg::Vec3Array> vertices (new osg::Vec3Array());
            osg::ref_ptr<osg::Vec4Array> colors (new osg::Vec4Array());
            
            for (int i=0; i<cloud_.points.size(); i++) {
                vertices->push_back (osg::Vec3 (cloud_.points[i].x, cloud_.points[i].y, cloud_.points[i].z));
                uint32_t rgb_val_;
                memcpy(&rgb_val_, &(cloud_.points[i].rgb), sizeof(uint32_t));
                
                uint32_t red,green,blue;
                blue=rgb_val_ & 0x000000ff;
                rgb_val_ = rgb_val_ >> 8;
                green=rgb_val_ & 0x000000ff;
                rgb_val_ = rgb_val_ >> 8;
                red=rgb_val_ & 0x000000ff;
                
                colors->push_back (osg::Vec4f ((float)red/255.0f, (float)green/255.0f, (float)blue/255.0f,1.0f));
            }
            
            geometry->setVertexArray (vertices.get());
            geometry->setColorArray (colors.get());
            geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
            
            geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,vertices->size()));
            
            cloudGeode_->addDrawable (geometry.get());
            osg::StateSet* state = geometry->getOrCreateStateSet();
            state->setMode( GL_LIGHTING,osg::StateAttribute::OFF);
            
            
            this->getAttachmentNode()->addChild(cloudGeode_.get());
            cloudGeode_->setName(std::string(id->s_name) + ".PointCloud");
        }
    }
    catch (pcl::InvalidConversionException e)
    {
        
        std::cout << "[PointCloud]: InvalidConversionException" << std::endl;
    }
    
}

// -----------------------------------------------------------------------------
    
void PointCloud::setDrawMode (DrawMode mode)
{
	if (this->drawMode_ != (int)mode)
	{
		this->drawMode_ = mode;
		BROADCAST(this, "si", "setDrawMode", getDrawMode());
	}
}

void PointCloud::setSpacing (float spacing)
{
	if (this->spacing_ != spacing)
	{
		this->spacing_ = spacing;
		BROADCAST(this, "sf", "setSpacing", this->spacing_);
	}
}

void PointCloud::setRandomCoeff (float randomCoeff)
{
	if (this->randomCoeff_ != randomCoeff)
	{
		this->randomCoeff_ = randomCoeff;
		BROADCAST(this, "sf", "setRandomCoeff", this->randomCoeff_);
	}
}
    
void PointCloud::setPointSize (float size)
{
    if (this->pointSize_ != size)
    {
        this->pointSize_ = size;
        BROADCAST(this, "sf", "setPointSize", this->pointSize_);
    }
}

void PointCloud::setColor (float r, float g, float b, float a)
{
	if (this->color_ != osg::Vec4(r,g,b,a))
	{
		this->color_ = osg::Vec4(r,g,b,a);
		BROADCAST(this, "sffff", "setColor", r, g, b, a);
	}
}

// -----------------------------------------------------------------------------
std::vector<lo_message> PointCloud::getState () const
{
    // inherit state from base class
	std::vector<lo_message> ret = GroupNode::getState();

    lo_message msg;
    
    msg = lo_message_new();
    lo_message_add(msg, "ss", "loadFile", path_.c_str());
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

    return ret;
}

} // end of namespace spin

