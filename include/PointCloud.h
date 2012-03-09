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

#ifndef PointCloud_H_
#define PointCloud_H_

#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "GroupNode.h"


namespace spin
{

// -----------------------------------------------------------------------------
/**
 * \brief Point Cloud renderer (can be used with Kinect data or any other .pcd
 * data
 */

class PointCloud : public GroupNode
{

public:

    PointCloud(SceneManager *sceneManager, char *initID);
    virtual ~PointCloud();
    
    enum DrawMode { NONE, POINTS, LINES, LINE_STRIP, LINE_LOOP, TRIANGLES, TRIANGLE_STRIP, TRIANGLE_FAN, QUADS, QUAD_STRIP, POLYGON, LIGHTPOINTS, BOXES };
    
    
    /**
     * update from kinect
     */
    virtual void callbackUpdate();

    void loadFile(const char* filename);
    
    void setDrawMode     (DrawMode mode);
    void setSpacing      (float spacing);
    void setRandomCoeff  (float randomCoeff);
    void setPointSize    (float pointsize);
    void setColor        (float red, float green, float blue, float alpha);

    int getDrawMode()      const { return (int)this->drawMode_; };
    float getSpacing()     const { return this->spacing_; };
    float getRandomCoeff() const { return this->randomCoeff_; };
    float getPointSize()   const { return this->pointSize_; };
    osg::Vec4 getColor()   const { return this->color_;  };


    /**
     * For each subclass of ReferencedNode, we override the getState() method to
     * fill the vector with the correct set of methods for this particular node
     */
    virtual std::vector<lo_message> getState() const;

    
private:
    
    //double depth_data[240][320];
    pcl::PointCloud<pcl::PointXYZRGB> cloud_;
    
    osg::ref_ptr<osg::MatrixTransform> cloudGroup;
    osg::ref_ptr<osg::Geode> cloudGeode_;

    DrawMode drawMode_;
    
    std::string path_;
    
    osg::ref_ptr<osg::MatrixTransform> shapeGroup;
    std::vector< osg::ref_ptr<osg::ShapeDrawable> > shapes;
    std::vector< osg::ref_ptr<osg::MatrixTransform> > shapeTransforms;
    
    float spacing_;
    float randomCoeff_;
    float pointSize_;
    osg::Vec4 color_;
    
    bool valid_;

};

} // end of namespace spin

#endif
