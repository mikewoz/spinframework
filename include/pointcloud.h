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

#include "config.h"

#include <memory>
#ifdef WITH_SHARED_VIDEO
#include <thread>
#endif

#include <osg/PositionAttitudeTransform>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osgSim/LightPointNode>
#include <osg/Timer>

#include "groupnode.h"

#ifdef WITH_PCL
#include <pcl/pcl_config.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#endif //WITH_PCL

#ifdef WITH_SHARED_VIDEO
#include <shmdata/any-data-reader.h>
#include "shmpointcloud.h"
#endif

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

    PointCloud(SceneManager *sceneManager, const char* initID);
    virtual ~PointCloud();
    
    enum DrawMode { NONE, POINTS, LINES, LINE_STRIP, LINE_LOOP, TRIANGLES, TRIANGLE_STRIP, TRIANGLE_FAN, QUADS, QUAD_STRIP, POLYGON, LIGHTPOINTS, BOXES, CUSTOM };
    enum ColorMode { NORMAL, OVERRIDE };
    virtual void debug();
    virtual void callbackUpdate(osg::NodeVisitor* nv);
    

    void setURI(const char* filename);

	
#ifdef WITH_PCL
#ifdef WITH_SHARED_VIDEO
	//static void shmCallback (
	//     shmdata_any_reader_t *reader,
    //     void *shmbuf,
    //     void *data,
    //     int data_size,
    //     unsigned long long timestamp,
    //     const char *type_description, void *user_data);
#endif
	
    void grabberCallback (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
    void applyFilters(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &rawCloud);

    osg::Vec3 getPos(unsigned int i);
    void getPos(unsigned int i, osg::Vec3& point);
    osg::Vec4f getColor(unsigned int i);
    void getColor(unsigned int i, osg::Vec4& color);

    virtual void updatePoints();
    virtual void draw();
#endif
    
    void setCustomNode   (const char* nodeID);
    void setDrawMode     (DrawMode mode);
    void setSpacing      (float spacing);
    void setRandomCoeff  (float randomCoeff);
    void setPointSize    (float pointsize);
    void setColor        (float red, float green, float blue, float alpha);
    void setColorMode    (ColorMode mode);

    /**
     * Set the voxelSize for the pcl::VoxelGrid filter, which downsamples points
     * to the nearest voxel in a 3D voxel grid. Think about a voxel grid as a
     * set of tiny 3D boxes in space.
     *
     * @param voxelSize in metres (default is 0.01, ie, 1cm). A value of 0 will
     * disable the VoxelGrid filter.
     */
    void setVoxelSize   (float voxelSize);
    
    /**
     * Set the minimum and maximum distance of valid points (in metres)
     */
    void setDistCrop   (float min, float max);


    const char* getCustomNode() const;
    int getDrawMode()       const { return (int)this->drawMode_; };
    float getSpacing()      const { return this->spacing_; };
    float getRandomCoeff()  const { return this->randomCoeff_; };
    float getPointSize()    const { return this->pointSize_; };
    osg::Vec4 getColor()    const { return this->color_;  };
    int getColorMode()      const { return (int)this->colorMode_; };
    float getFilterSize()   const { return this->voxelSize_; };
    osg::Vec2 getDistCrop() const { return this->distCrop_; };

    virtual std::vector<lo_message> getState() const;
    


private:
    
#ifdef WITH_PCL
    pcl::Grabber* grabber_;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOrig_; // in the case of a file
    #if PCL_VERSION_HIGHER_THAN_1_6
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> *decoder_;
    #else
	pcl::octree::PointCloudCompression<pcl::PointXYZRGBA> *decoder_;
    #endif
#endif

#ifdef WITH_SHARED_VIDEO
	//shmdata_any_reader_t *shmReader_;
    std::shared_ptr<ShmPointCloud<pcl::PointXYZRGBA>> shmPointCloud_;
    std::shared_ptr<std::thread> shmThread_;
    bool shmIsRunning;
#endif
	
    t_symbol* customNode_;
    
    osg::ref_ptr<osg::PositionAttitudeTransform> cloudGroup_;
    osg::ref_ptr<osgSim::LightPointNode> lightPointNode_;
    osg::ref_ptr<osg::Geode> cloudGeode_;
    std::vector<osg::MatrixTransform*> xforms_;
    
    DrawMode drawMode_;
    
    std::string path_;
    
    int maxPoints_; // for kinect, or any time you need to reserve points
    
    float spacing_;
    float randomCoeff_;
    float pointSize_;
    osg::Vec4 color_;
    ColorMode colorMode_;
    float voxelSize_;
    osg::Vec2 distCrop_;
    
    bool redrawFlag_;
    bool updateFlag_;
    
    float framerate_;

};

} // end of namespace spin



#endif
