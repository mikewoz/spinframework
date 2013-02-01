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


#include "config.h"

#include <chrono>

#include <osg/Geode>
#include <osg/BlendColor>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osgUtil/Optimizer>
#include <osg/Geometry>
#include <osg/Point>
#include <osg/PolygonMode>
#include <osgUtil/SmoothingVisitor>
#include <osgSim/LightPointNode>
#include <osgDB/FileNameUtils>

#include "pointcloud.h"
#include "scenemanager.h"
#include "spinapp.h"
#include "spinbasecontext.h"
#include "osgutil.h"

using namespace std;

//using namespace std;
extern pthread_mutex_t sceneMutex;

// TODO: grabberMutex should be a member right? .. what if there are >1 devices?
#ifdef WITH_PCL
static boost::mutex grabberMutex;
#endif

namespace spin
{

// -----------------------------------------------------------------------------
// constructor:
PointCloud::PointCloud (SceneManager *sceneManager, const char* initID) : GroupNode(sceneManager, initID)
{
	this->setName(this->getID() + ".PointCloud");
	this->setNodeType("PointCloud");

	drawMode_ = POINTS;
    
    updateFlag_ = false;
    redrawFlag_ = false;
    
    maxPoints_ = 0;
    framerate_ = 0;
    path_ = "NULL";
    customNode_ = gensym("NULL");
    
    color_ = osg::Vec4(1.0,1.0,1.0,1.0);
    colorMode_ = NORMAL;
    spacing_ = 1.0;
    randomCoeff_ = 0.0;
    pointSize_ = 1.0;
    
    voxelSize_ = 0.01f;
    distCrop_ = osg::Vec2(0.0,10.0);
    
#ifdef WITH_PCL
	#ifdef WITH_SHARED_VIDEO
	//shmReader_ = 0;
	#endif
    grabber_ = 0;
    #if PCL_VERSION_HIGHER_THAN_1_6
    decoder_ = new pcl::io::OctreePointCloudCompression< pcl::PointXYZRGBA >();
    #else
	decoder_ = new pcl::octree::PointCloudCompression< pcl::PointXYZRGBA >();
    #endif
#endif
    
}

// destructor
PointCloud::~PointCloud()
{
#ifdef WITH_PCL
    if (grabber_)
    {
        grabber_->stop();
        grabber_ = 0;
    }
    
	#ifdef WITH_SHARED_VIDEO
    if (shmPointCloud_.get())
    {
        shmIsRunning = false;
        shmThread_->join();
    }
    #endif
#endif
}
// -----------------------------------------------------------------------------

void PointCloud::debug()
{
    GroupNode::debug();
    
#ifdef WITH_PCL
    boost::mutex::scoped_lock lock(grabberMutex);
    if (cloud_)
    {
        std::cout << "   " << (int)cloud_->points.size() << " of " << maxPoints_ << " displayed" << std::endl;
        std::cout << "   Avg framerate: " << framerate_ << std::endl;
    }
    else
    {
        std::cout << "   No valid point cloud loaded" << std::endl;
    }
#else
    std::cout << "   SPIN was not compiled with PointCloud support; rendering disabled." << std::endl;
#endif
}

// -----------------------------------------------------------------------------
void PointCloud::callbackUpdate(osg::NodeVisitor* nv)
{

#ifdef WITH_PCL
    // A "redraw" will destroy the current subgraph and re-create it.
    // This is done, for example, when the type of geometry is changed (eg, from
    // lightpoints to lines). Don't do this too often!
    if (redrawFlag_)
    {
        this->draw();
        redrawFlag_ = false;
        updateFlag_ = true;
    }
    
    // an "update" will just update positions, colors, spacing, etc of the
    // points (eg, when the grabberCallback gets a new frame from the Kinect)
    if (updateFlag_)
    {
        this->updatePoints();
        updateFlag_ = false;
    }


#endif
    
    GroupNode::callbackUpdate(nv);
}


// -----------------------------------------------------------------------------
#ifdef WITH_PCL

#ifdef WITH_SHARED_VIDEO 
//void PointCloud::shmCallback (
//         shmdata_any_reader_t *reader,
//         void *shmbuf,
//         void *data,
//         int data_size,
//         unsigned long long timestamp,
//         const char *type_description, void *user_data)
//{
//
//	PointCloud *node = (PointCloud*)(user_data);
//
//	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGBA>());
//	
// 	// stringstream to store compressed point cloud
//	std::stringstream compressedData;
//	compressedData.write ((const char *) data, data_size);
//	
//  	// decode:
//	node->decoder_->decodePointCloud(compressedData,cloudOut);
//
//	std::cout << "shmCallback got " << cloudOut->points.size() << " points" << std::endl;
//	
//	// update stored pointcloud:
//	{
//		// Just write the pointcloud to our local instance:
//		/*
//		boost::mutex::scoped_lock lock (grabberMutex);
//		node->cloud_.swap(cloudOut);
//		*/
//		
//		// OR call applyFilters:
//		node->applyFilters(cloudOut);
//	}
//
//	// TODO: we should only set updateFlag_ here and try to draw only once
//	// when we get the first frame, right? Unless, the shm data can change size?
//	//node->redrawFlag_ = true;
//	node->updateFlag_ = true;
//	
//	shmdata_any_reader_free (shmbuf);
//}
#endif

//#if PCL_MAJOR_VERSION>1 && PCL_MINOR_VERSION>5
void PointCloud::grabberCallback (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &rawCloud)
//#else
//void PointCloud::grabberCallback (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &rawCloud)
//#endif
{
	
    // DEBUG INFO
	/*
    static unsigned count = 0;
    static osg::Timer_t lastTick = osg::Timer::instance()->tick();
    if (++count == 30)
    {
        osg::Timer_t tick = osg::Timer::instance()->tick();
        framerate_ = count / osg::Timer::instance()->delta_s(lastTick,tick);
        float center = rawCloud->points[(rawCloud->width >> 1) * (rawCloud->height + 1)].z;
        std::cout << rawCloud->points.size() << " points. Distance of center pixel=" << center << "mm. Average framerate: " << framerate_ << " Hz" <<  std::endl;
        count = 0;
        lastTick = tick;
    }
	*/
    
    // If the redraw flag is set, our geometry is about to be destroyed anyway,
    // so let's just exit and wait. The cloud needs to exist before we can
    // update it.
    if (redrawFlag_) return;
        
    applyFilters(rawCloud);

    // Set the flag so that we update during the next traversal:
    updateFlag_ = true;
}

void PointCloud::applyFilters(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &rawCloud)
{
    // First apply crop filter along depth (z) axis:   
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PassThrough<pcl::PointXYZRGBA> pass;
    pass.setInputCloud(rawCloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (distCrop_.x(), distCrop_.y());
    //pass.setFilterLimitsNegative (true);
    pass.filter(*cloudFiltered);
    
    // Now we apply a VoxelGrid filter to reduce the number of points
    if (voxelSize_>0)
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
        sor.setInputCloud(cloudFiltered);
        sor.setLeafSize(voxelSize_, voxelSize_, voxelSize_);
        sor.filter(*cloud);
        
        // now set out member pointer with a mutex
        {
            boost::mutex::scoped_lock lock (grabberMutex);
            cloud_.swap(cloud);
        }
    }
    else 
    {
        boost::mutex::scoped_lock lock (grabberMutex);
        cloud_.swap(cloudFiltered);
    }
}

osg::Vec3 PointCloud::getPos(unsigned int i)
{
    return osg::Vec3 (cloud_->points[i].x*spacing_, cloud_->points[i].y*spacing_, cloud_->points[i].z*spacing_) + randomVec3()*randomCoeff_;
}

void PointCloud::getPos(unsigned int i, osg::Vec3& point)
{
    point.set(cloud_->points[i].x*spacing_, cloud_->points[i].y*spacing_, cloud_->points[i].z*spacing_);
}

osg::Vec4f PointCloud::getColor(unsigned int i)
{
    if (colorMode_==OVERRIDE) return color_;

    uint32_t rgba_val_;
    memcpy(&rgba_val_, &(cloud_->points[i].rgba), sizeof(uint32_t));
    
    uint32_t red,green,blue;
    blue=rgba_val_ & 0x000000ff;
    rgba_val_ = rgba_val_ >> 8;
    green=rgba_val_ & 0x000000ff;
    rgba_val_ = rgba_val_ >> 8;
    red=rgba_val_ & 0x000000ff;
    
    return osg::Vec4f ((float)red/255.0f, (float)green/255.0f, (float)blue/255.0f,1.0f);
}

void PointCloud::getColor(unsigned int i, osg::Vec4& color)
{
    if (colorMode_==OVERRIDE)
        color = color_;

    uint32_t rgba_val_;
    memcpy(&rgba_val_, &(cloud_->points[i].rgba), sizeof(uint32_t));
    
    uint32_t red,green,blue;
    blue=rgba_val_ & 0x000000ff;
    rgba_val_ = rgba_val_ >> 8;
    green=rgba_val_ & 0x000000ff;
    rgba_val_ = rgba_val_ >> 8;
    red=rgba_val_ & 0x000000ff;

    color.set((float)red/255.0f, (float)green/255.0f, (float)blue/255.0f,1.0f);
}

void PointCloud::updatePoints()
{
    if (spinApp::Instance().getContext()->isServer()) return;
    if (redrawFlag_ || !cloud_) return;

    boost::mutex::scoped_lock lock(grabberMutex);
    
    //std::cout << "doing update. cloudsize=" << cloud_->points.size() << " maxsize=" << maxPoints_ << std::endl;
    //if (lightPointNode_.valid()) std::cout << "num lightpoints=" << lightPointNode_->getNumLightPoints() << std::endl;    
        
    if ((drawMode_>=POINTS) && (drawMode_<=POLYGON))
    {
        cloudGeode_->removeDrawables(0,cloudGeode_->getNumDrawables());
        
        osg::Vec3Array* verts = new osg::Vec3Array();
        osg::Vec4Array* colors = new osg::Vec4Array();
        
        verts->resize(cloud_->points.size());
        colors->resize(cloud_->points.size());

        for (unsigned int i=0; i<cloud_->points.size(); i++)
        {
            this->getPos(i, (*verts)[i]);
            this->getColor(i, (*colors)[i]);
        }

        osg::Geometry *geom = new osg::Geometry();
        switch (drawMode_)
        {
            case LINES:
                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,cloud_->points.size()));
                break;
            case LINE_STRIP:
                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP,0,cloud_->points.size()));
                break;
            case LINE_LOOP:
                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP,0,cloud_->points.size()));
                break;
            case TRIANGLES:
                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES,0,cloud_->points.size()));
                break;
            case TRIANGLE_STRIP:
                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLE_STRIP,0,cloud_->points.size()));
                break;
            case TRIANGLE_FAN:
                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLE_FAN,0,cloud_->points.size()));
                break;
            case QUADS:
                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,cloud_->points.size()));
                break;
            case QUAD_STRIP:
                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUAD_STRIP,0,cloud_->points.size()));
                break;
            case POLYGON:
                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POLYGON,0,cloud_->points.size()));
                break;
            default: // POINTS
                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,cloud_->points.size()));
                
                osg::Point *point = new osg::Point();
                point->setSize(pointSize_*2);
                cloudGeode_->getOrCreateStateSet()->setAttribute(point);

                break;
        }
        geom->setVertexArray(verts);
        geom->setColorArray(colors);
        geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
        cloudGeode_->addDrawable( geom );
    }
    
    else if ((drawMode_==LIGHTPOINTS) && lightPointNode_.valid() && lightPointNode_->getNumLightPoints())
    {        
        unsigned int i=0;
        for (; i<cloud_->points.size(); i++)
        {
            osgSim::LightPoint& lp = lightPointNode_->getLightPoint(i);
            lp._on = true;
            this->getColor(i, lp._color);
            this->getPos(i, lp._position);
            lp._radius = pointSize_/1000.0;
        }
        for (; i<maxPoints_; i++)
        {
            // deactivate any filtered out light points (recall, we reserve the
            // maximum for the camera resolution)
            osgSim::LightPoint& lp = lightPointNode_->getLightPoint(i);
            lp._on = false;
        }
    }
    
    else if ( (drawMode_ == BOXES) || (drawMode_ == CUSTOM) )
    {
        unsigned int i=0;
        for (; i<cloud_->points.size(); i++)
        {
            //xforms_[i]->setMatrix(osg::Matrix::translate(this->getPos(i)));
            
            float scale = pointSize_/200.0;
            xforms_[i]->setMatrix(osg::Matrix::scale(osg::Vec3(scale,scale,scale))*osg::Matrix::translate(this->getPos(i)));
            
            xforms_[i]->setNodeMask(0xffffffff);
            
            // doesn't work:
            //osg::BlendColor* bc = new osg::BlendColor(this->getColor(i));
            //xform->getOrCreateStateSet()->setAttributeAndModes(bc, osg::StateAttribute::ON); 
            //xform->getOrCreateStateSet()->setMode(osg::StateAttribute::BLENDCOLOR, osg::StateAttribute::ON); 
        }
        for (; i<maxPoints_; i++)
        {
            xforms_[i]->setNodeMask(0x0);
        }            

        // TODO: SmoothingVisitor? Optimizer?
    }

}


        
void PointCloud::draw()
{
    // Do NOT call this method too often. The only real time you want to call
    // this is if the *number* of vertices changes, or if the geometry needs to
    // be drawn in a different way. If it's just positions, colors, spacing, etc
    // that is changing, then use updatePoints().

    if (!sceneManager_->isGraphical()) return;
    
    //std::cout << "PointCloud redraw. cloud_ size = " << cloud_->size() << " maxPoints=" << maxPoints_ << std::endl;

    // remove previous pointcloud:
    if (cloudGroup_.valid() && getAttachmentNode()->containsNode(cloudGroup_.get()))
    {
        getAttachmentNode()->removeChild(cloudGroup_.get());
        cloudGroup_ = NULL;

        if (lightPointNode_.valid()) lightPointNode_ = NULL;
        if (cloudGeode_.valid()) cloudGeode_ = NULL;
        xforms_.clear();
    }
    
    if (maxPoints_<=0) return;

    cloudGroup_ = new osg::PositionAttitudeTransform();
    cloudGroup_->setAttitude(osg::Quat(-osg::PI_2, osg::X_AXIS));
    cloudGroup_->setName(this->getID() + ".CloudGroup");

    
    if ((drawMode_>=POINTS) && (drawMode_<=POLYGON))
    {
        cloudGeode_ = new osg::Geode();
        cloudGeode_->setName(this->getID() + ".CloudGeode");
        osg::StateSet *ss = cloudGeode_->getOrCreateStateSet();
		ss->setMode( GL_LIGHTING, osg::StateAttribute::OFF);
        cloudGroup_->addChild(cloudGeode_);
    
        // the geometry will be created dynamically in updatePoints()
    }
    
    else if (drawMode_ == LIGHTPOINTS)
    {
        lightPointNode_ = new osgSim::LightPointNode();
        lightPointNode_->getLightPointList().reserve(maxPoints_);
        
        for (unsigned int i=0; i<maxPoints_; i++)
        {
            osgSim::LightPoint lp;
            lp._on = false;
            lightPointNode_->addLightPoint(lp);
        }
        
        cloudGroup_->addChild(lightPointNode_);
    }
        
    else if (drawMode_ == BOXES)
    {
        osg::Geode* geode = new osg::Geode();
        
        osg::TessellationHints* hints = new osg::TessellationHints;
        hints->setDetailRatio(1);
        hints->setCreateBackFace(false);
        
        //osg::Box* box = new osg::Box(osg::Vec3(0,0,0), pointSize_/500.0);
        osg::Box* box = new osg::Box(osg::Vec3(0,0,0), pointSize_/2);
        osg::ShapeDrawable* boxDrawable = new osg::ShapeDrawable(box,hints);
        geode->addDrawable(boxDrawable);
        
        for (unsigned int i=0; i<maxPoints_; i++)
        {
            osg::MatrixTransform *xform = new osg::MatrixTransform();

            xform->setNodeMask(0x0);
            xform->addChild(geode);
            cloudGroup_->addChild(xform);
            xforms_.push_back(xform);
        }
    }
    
    /*
    else if (drawMode_ == SQUARES)
    {
        // TODO: Billboarded planes
    }
    */
    
    else if (drawMode_ == CUSTOM)
    {
        ReferencedNode *n = dynamic_cast<ReferencedNode*>(customNode_->s_thing);

        if (n)
        {
            for (unsigned int i=0; i<maxPoints_; i++)
            {
                osg::MatrixTransform *xform = new osg::MatrixTransform();
                
                xform->setNodeMask(0x0);
                xform->addChild(n);
                cloudGroup_->addChild(xform);
                xforms_.push_back(xform);
            }
        }
    }
    
    this->getAttachmentNode()->addChild(cloudGroup_.get());
}

#endif


// -----------------------------------------------------------------------------

void PointCloud::setURI(const char* filename)
{
    // ignore this msg if we already have this filename:
    if (std::string(filename)==path_) return;
    
    path_ = std::string(filename);

    // if this is the server, then just broadcast the path and return:
    if (!sceneManager_->isGraphical())
    {
        BROADCAST(this, "ss", "setURI", filename);
        return;
    }
    
    // otherwise, this is a client (viewer), so:

#ifdef WITH_PCL

    // first, check if the old path was a kinect, and destroy it accordingly:
    if (grabber_)
    {
        grabber_->stop();
        grabber_ = 0;
    }
#ifdef WITH_SHARED_VIDEO
	//if (shmReader_)
	//{
	//	shmdata_any_reader_close(shmReader_);
	//	shmReader_ = 0;
	//}
#endif
	
    cloudOrig_.reset();
            
    path_ = filename;
    maxPoints_ = 0;
    
    bool success = false;

    
    if (path_ == "NULL")
    {
        // do nothing
        return;
    }

	else if (std::string(path_).find("shm://") != std::string::npos)
	{
		
		std::string shmPath = path_.substr(6);
		std::cout << "Connecting to shmdata path: " << shmPath << std::endl;
		
#ifdef WITH_SHARED_VIDEO
		//shmReader_ = shmdata_any_reader_init();
		//if (1)//(verbose)
		//	shmdata_any_reader_set_debug(shmReader_, SHMDATA_ENABLE_DEBUG);
		//else
		//	shmdata_any_reader_set_debug(shmReader_, SHMDATA_DISABLE_DEBUG);

		//shmdata_any_reader_set_on_data_handler(shmReader_, &PointCloud::shmCallback, this);
  		//shmdata_any_reader_set_data_type(shmReader_, "application/x-pcd-compressed");
  		//shmdata_any_reader_start(shmReader_, shmPath.c_str());

        shmPointCloud_.reset(new ShmPointCloud<pcl::PointXYZRGBA>(shmPath.c_str(), false));

        // We create a thread to handle the clouds from the shmData
        shmIsRunning = true;
        shmThread_.reset(new std::thread([&] ()
        {
            struct timespec sleepTime;
            sleepTime.tv_sec = 0;
            sleepTime.tv_nsec = 1e5;

            while (shmIsRunning)
            {
                if (shmPointCloud_->isUpdated())
                {
                    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGBA>());
                    shmPointCloud_->getCloud(cloudOut);

                    std::cout << "shmCallback got " << cloudOut->points.size() << " points" << std::endl;
    		        applyFilters(cloudOut);

                    updateFlag_ = true;
                }

                nanosleep(&sleepTime, NULL);
            }
        } ));

#endif

		maxPoints_ = 76800;
		success = true;
	}
    else if (std::string(path_).find("kinect://") != std::string::npos)
    {
        // Kinect or any other dynamic point cloud needs to reserve a maximum
        // number of points. These get added to the scenegraph only ONCE during
        // the draw() method, which is thread-safe.
        //
        // In the callback from the Kinect, we should never ask for a redraw.
        // that is too slow. Instead, we set the updateFlag to call
        // updatePoints() method
                
        try
        {
            //std::string kinectID = "#1";
            std::string kinectID = "#"+path_.substr(9);

            std::cout << "Connecting to Kinect " << kinectID << ". This could take several seconds." << std::endl;
            
            grabber_ = new pcl::OpenNIGrabber(kinectID, pcl::OpenNIGrabber::OpenNI_QVGA_30Hz, pcl::OpenNIGrabber::OpenNI_QVGA_30Hz);

            // make callback function from member function
            boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
              boost::bind (&PointCloud::grabberCallback, this, _1);

            // connect callback function for desired signal
            boost::signals2::connection c = grabber_->registerCallback (f);
           
            // For now, we're hardcoding OpenNI_QVGA_30Hz, which is 320x240
            // so reserve 76800 points.
            maxPoints_ = 76800;
            
            // need to set the redrawFlag_ befoe we start the grabber, since
            // updates don't happen when the redrawFlag_ is set.
            redrawFlag_ = true;
        
            // start receiving point clouds
            grabber_->start();
            
            framerate_ = 0;
            success = true;
        }
        #if PCL_VERSION_HIGHER_THAN_1_6
        catch (pcl::PCLException e)
        #else
        catch (pcl::PCLIOException e)
        #endif
        {
            std::cout << "[PointCloud]: Error connecting to Kinect; perhaps it is already being used? ... " << e.detailedMessage() << std::endl;
        }
    }
    
    else
    {
        // load the file:
        std::string absPath = getAbsolutePath(std::string(path_));
        std::string ext = osgDB::getLowerCaseFileExtension(absPath);
        
        try
        {
            if (ext=="cpc")
            {
                std::stringstream compressedData;
                #if PCL_VERSION_HIGHER_THAN_1_6
                pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> *pointCloudDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>();
                #else
                pcl::octree::PointCloudCompression<pcl::PointXYZRGBA> *pointCloudDecoder = new pcl::octree::PointCloudCompression<pcl::PointXYZRGBA>();
                #endif
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmpCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

                std::ifstream readCompressedFile(absPath.c_str());
                compressedData << readCompressedFile.rdbuf();
                
                pointCloudDecoder->decodePointCloud(compressedData, tmpCloud);
                
                {
                    boost::mutex::scoped_lock lock (grabberMutex);
                    cloudOrig_.swap(tmpCloud);
                }
                maxPoints_ = cloudOrig_->points.size();
                success = true;
            }
            
            else // if (ext=="pcd")
            {
                pcl::PointCloud<pcl::PointXYZRGBA> tmpCloud;
                if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(absPath, tmpCloud) != -1)  
                {            
                    std::cout << tmpCloud.points.size() << " PointXYZRGBA " << std::endl;

                    {
                        boost::mutex::scoped_lock lock (grabberMutex);
                        cloudOrig_ = tmpCloud.makeShared(); // note: deep copy (fix?)
                    }
                    maxPoints_ = cloudOrig_->points.size();
                    success = true;
                }
            }
        }
        catch (pcl::InvalidConversionException e)
        {
            std::cout << "[PointCloud]: Couldn't load " << path_ <<  "in XYZRGBA format. Trying XYZ." << std::endl;
        }

        if (!success)
        try
        {
            pcl::PointCloud<pcl::PointXYZ> tmpCloud;
            if (pcl::io::loadPCDFile(absPath, tmpCloud) != -1)  
            {     
                // now we need to convert to XYZRGBA:
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr converted(new pcl::PointCloud<pcl::PointXYZRGBA>);
                converted->points.resize(tmpCloud.size());
                for (size_t i = 0; i < tmpCloud.points.size(); i++)
                {
                    converted->points[i].x = tmpCloud.points[i].x;
                    converted->points[i].y = tmpCloud.points[i].y;
                    converted->points[i].z = tmpCloud.points[i].z;
                }
                
                {
                    boost::mutex::scoped_lock lock (grabberMutex);
                    cloudOrig_.swap(converted);
                }
                maxPoints_ = cloudOrig_->points.size();
                success = true;
            }
        }
        catch (pcl::InvalidConversionException e)
        {
            std::cout << "[PointCloud]: Couldn't load .pcd ing XYZ format." << std::endl;
        }
    }
    
    
    if (success)
    {    
        std::cout << "[PointCloud]: Loaded " << maxPoints_ << " data points from " << path_ << std::endl;
        this->applyFilters(cloudOrig_);
        redrawFlag_ = true;
    } else
    {
        std::cout << "[PointCloud]: Couldn't read file " << path_ << std::endl;
    }
    
#endif
}

// -----------------------------------------------------------------------------

void PointCloud::setCustomNode(const char* nodeID)
{
    ReferencedNode *n = sceneManager_->getNode(nodeID);
    if (n)
    {
        customNode_ = gensym(nodeID);
        BROADCAST(this, "ss", "setCustomNode", nodeID);
        
        redrawFlag_ = true;
    }
    else if (std::string(nodeID)!="NULL")
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
        updateFlag_ = true;
	}
}

void PointCloud::setRandomCoeff (float randomCoeff)
{
	if (this->randomCoeff_ != randomCoeff)
	{
		this->randomCoeff_ = randomCoeff;
		BROADCAST(this, "sf", "setRandomCoeff", this->randomCoeff_);
        updateFlag_ = true;
	}
}
    
void PointCloud::setPointSize (float size)
{
    if (this->pointSize_ != size)
    {
        this->pointSize_ = size;
        BROADCAST(this, "sf", "setPointSize", this->pointSize_);
        updateFlag_ = true;
    }
}

void PointCloud::setVoxelSize (float voxelSize)
{
    if (this->voxelSize_ != voxelSize)
    {
        this->voxelSize_ = voxelSize;
        BROADCAST(this, "sf", "setVoxelSize", this->voxelSize_);
        
#ifdef WITH_PCL
        if (cloudOrig_)
        {
            // if cloudOrig_ is valid, it means this point cloud was loaded
            // from file, and we need to apply filters:
            this->applyFilters(cloudOrig_);
        }
#endif

        updateFlag_ = true;
    }
}

void PointCloud::setDistCrop (float min, float max)
{
    if ((min!=distCrop_.x()) || (max!=distCrop_.y()))
    {
        distCrop_ = osg::Vec2(min,max);
        BROADCAST(this, "sff", "setDistCrop", min, max);

#ifdef WITH_PCL
        if (cloudOrig_)
        {
            // if cloudOrig_ is valid, it means this point cloud was loaded
            // from file, and we need to apply filters:
            this->applyFilters(cloudOrig_);
        }
#endif

        updateFlag_ = true;
    }
}

void PointCloud::setColor (float r, float g, float b, float a)
{
	if (this->color_ != osg::Vec4(r,g,b,a))
	{
		this->color_ = osg::Vec4(r,g,b,a);
		BROADCAST(this, "sffff", "setColor", r, g, b, a);
        updateFlag_ = true;
	}
}

void PointCloud::setColorMode (ColorMode mode)
{
	if (this->colorMode_ != (int)mode)
	{
		this->colorMode_ = mode;
		BROADCAST(this, "si", "setColorMode", getColorMode());
        
        updateFlag_ = true;
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
    lo_message_add(msg, "sf", "setVoxelSize", getFilterSize());
    ret.push_back(msg);

    msg = lo_message_new();
    osg::Vec2 v2 = getDistCrop();
    lo_message_add(msg, "sff", "setDistCrop", v2.x(), v2.y());
    ret.push_back(msg);

    msg = lo_message_new();
    osg::Vec4 v3 = this->getColor();
    lo_message_add(msg, "sffff", "setColor", v3.x(), v3.y(), v3.z(), v3.w());
 
    msg = lo_message_new();
    lo_message_add(msg, "si", "setColorMode", getColorMode());
    ret.push_back(msg);
    
    msg = lo_message_new();
    lo_message_add(msg, "ss", "setURI", path_.c_str());
    ret.push_back(msg);


    return ret;
}

} // end of namespace spin
