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

#include <pthread.h>

#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osgUtil/Optimizer>
#include <osg/Geometry>
#include <osgUtil/SmoothingVisitor>
#include <osgSim/LightPointNode>


#include "KinectCloud.h"
#include "SceneManager.h"
#include "spinApp.h"
#include "spinBaseContext.h"
#include "osgUtil.h"

//volatile int die = 0;

double depth_data[240][320];

pthread_mutex_t freenectMutex = PTHREAD_MUTEX_INITIALIZER;

namespace spin
{

// -----------------------------------------------------------------------------
// constructor:
KinectCloud::KinectCloud (SceneManager *sceneManager, const char* initID) : GroupNode(sceneManager, initID)
{
	this->setName(this->getID() + ".KinectCloud");
	this->setNodeType("KinectCloud");

	drawMode_ = LIGHTPOINTS;
    
    valid_ = false;
	color_ = osg::Vec4(1.0,1.0,0.0,1.0);
    spacing_ = 10.0;
    randomCoeff_ = 0.0;
    pointSize_ = 1.0;

    for (int i=0; i<240; i++)
        for (int j=0; j<320; j++)
            depth_data[i][j] = 0;
    
    
    if (sceneManager_->isGraphical())
    {
        
        osg::TessellationHints* hints = new osg::TessellationHints;
        hints->setDetailRatio(GENERIC_SHAPE_RESOLUTION);
        shapeGroup = new osg::MatrixTransform();
        shapeGroup->setMatrix(osg::Matrix::scale(0.002, 0.002, 0.002)*osg::Matrix::rotate(-osg::PI_2,osg::X_AXIS));
        for (int i=0; i<240; i++)
        {
            for (int j=0; j<320; j++)
            {
                osg::ShapeDrawable *shp = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0.0f,0.0f,0.0f), AS_UNIT_SCALE), hints);
                osg::MatrixTransform *xform = new osg::MatrixTransform();
                
                osg::Geode *geode = new osg::Geode();
                geode->addDrawable(shp);
                xform->addChild(geode);
                shapeGroup->addChild(xform);
                
                shapes.push_back(shp);
                shapeTransforms.push_back(xform);
            }
        }

        if (freenect_init(&f_ctx, NULL) < 0)
        {
            std::cout<<"KinectCloud["<<this->getID()<<"] freenect_init() failed" << std::endl;
            valid_ = false;
        }
        
        freenect_set_log_level(f_ctx, FREENECT_LOG_DEBUG);
        if (freenect_open_device(f_ctx, &f_dev, 0) < 0)
        {
            std::cout<<"KinectCloud["<<this->getID()<<"] Could not open device" << std::endl;
            valid_ = false;
        }
        
        int res = pthread_create(&freenectThread, NULL, freenectThreadFunction, this);
        if (res)
        {
            std::cout << "KinectCloud["<<this->getID()<<"] pthread_create failed" << std::endl;
            valid_ = false;
        }
        
        valid_ = true;
    }

}

// destructor
KinectCloud::~KinectCloud()
{
    // TODO: need to clean up freenect (currently can't create a new node)
}

// -----------------------------------------------------------------------------
void KinectCloud::callbackUpdate(osg::NodeVisitor* nv)
{
    GroupNode::callbackUpdate(nv);
    
    if (getAttachmentNode()->containsNode(cloudGroup.get()))
    {
        getAttachmentNode()->removeChild(cloudGroup.get());
        cloudGroup = NULL;
    }
    
    // exit if the kinect is not connected / available:
    if (!this->valid_) return;
    
    
    // special group for shapes:
    if ((drawMode_==BOXES) && (!getAttachmentNode()->containsNode(shapeGroup.get())))
    {
        getAttachmentNode()->addChild(shapeGroup.get());
    }
    else if ((drawMode_!=BOXES) && (getAttachmentNode()->containsNode(shapeGroup.get())))
    {
        getAttachmentNode()->removeChild(shapeGroup.get());
    }

    
    cloudGroup = new osg::MatrixTransform;
    
    //cloudGroup->setDataVariance(osg::Object::STATIC);
    cloudGroup->setMatrix(osg::Matrix::scale(0.002, 0.002, 0.002)*osg::Matrix::rotate(-osg::PI_2,osg::X_AXIS));
    
    int noStepsX = 320;
    int noStepsY = 240;
    
    pthread_mutex_lock(&freenectMutex);
    
    osg::ref_ptr<osg::Vec3Array> vertexArray = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec4Array> colorArray = new osg::Vec4Array;
    vertexArray->clear();
    colorArray->clear();
    for (int j=0; j<noStepsY; ++j)
    {
        for (unsigned int i=0; i<noStepsX; ++i)
        {
            float z = depth_data[j][i] * 14.0;
            float y = (j-120.0) * (z + -spacing_) * 0.005;
            float x = (i-160.0) * (z + -spacing_) * 0.005;
            if (depth_data[j][i] <= ( 100.0/3.33 ))
                continue;
            vertexArray->push_back(osg::Vec3(x,y,z));
            colorArray->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
        }
    }
    
    pthread_mutex_unlock(&freenectMutex);
    
    
    osg::TessellationHints* hints = new osg::TessellationHints;
    hints->setDetailRatio(GENERIC_SHAPE_RESOLUTION);
    

    if ((drawMode_>POINTS) && (drawMode_<=POLYGON))
    {
        osg::Geode* geode = new osg::Geode();
		osg::Geometry* geom = new osg::Geometry();
        geom->setVertexArray(vertexArray.get());
        
        switch (drawMode_)
        {
            case LINES:
                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,vertexArray->size()));
                break;
            case LINE_STRIP:
                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP,0,vertexArray->size()));
                break;
            case LINE_LOOP:
                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP,0,vertexArray->size()));
                break;
            case TRIANGLES:
                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES,0,vertexArray->size()));
                break;
            case TRIANGLE_STRIP:
                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLE_STRIP,0,vertexArray->size()));
                break;
            case TRIANGLE_FAN:
                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLE_FAN,0,vertexArray->size()));
                break;
            case QUADS:
                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,vertexArray->size()));
                break;
            case QUAD_STRIP:
                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUAD_STRIP,0,vertexArray->size()));
                break;
            case POLYGON:
                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POLYGON,0,vertexArray->size()));
                break;
            default: // POINTS
                geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,vertexArray->size()));
                break;
        }
        
        geode->addDrawable( geom );
        geom->setColorArray(colorArray);
        geom->setColorBinding(osg::Geometry::BIND_OVERALL);
        
        osg::StateSet *ss = geode->getOrCreateStateSet();
		ss->setMode( GL_LIGHTING, osg::StateAttribute::OFF);
        
        cloudGroup->addChild(geode);
    }
    
    else if (drawMode_ == LIGHTPOINTS)
    {
        osgSim::LightPointNode* lpn = new osgSim::LightPointNode;
        lpn->getLightPointList().reserve(vertexArray->size());
        
        for (unsigned int i=0; i<vertexArray->size(); i++)
        {
            osgSim::LightPoint lp;
            lp._color = colorArray->at(i);
            lp._position = vertexArray->at(i);
            lp._radius = pointSize_;
            lpn->addLightPoint(lp);
        }
        cloudGroup->addChild(lpn);
    }
        
    else if (drawMode_ == BOXES)
    {
        /*
        osg::Geode* geode = new osg::Geode();
        
        for (unsigned int i=0; i<vertexArray->size(); i++)
        {
            osg::Box* box = new osg::Box(vertexArray->at(i), pointSize_);
            osg::ShapeDrawable* boxDrawable = new osg::ShapeDrawable(box,hints);
            boxDrawable->setColor(colorArray->at(i));
            geode->addDrawable(boxDrawable);
        }
        cloudGroup->addChild(geode);
         */
        
        for (unsigned int i=0; i<vertexArray->size(); i++)
        {
            shapes[i]->setColor(colorArray->at(i));
            shapeTransforms[i]->setMatrix(osg::Matrix::scale(osg::Vec3(pointSize_,pointSize_,pointSize_))*osg::Matrix::translate(vertexArray->at(i)));
        }
        for (unsigned int i=vertexArray->size(); i<noStepsX*noStepsY; i++)
        {
            shapeTransforms[i]->setMatrix(osg::Matrix::scale(osg::Vec3(0,0,0)));
        }
    }
    
    // TODO: SmoothingVisitor? Optimizer?
    
    getAttachmentNode()->addChild(cloudGroup);
}
    
static int frame = 0;
void KinectCloud::depthCallback(freenect_device *dev, void *v_depth, uint32_t timestamp)
{
    uint16_t *depth = ( uint16_t * ) v_depth;
    pthread_mutex_lock(&freenectMutex);
    for ( int i=0; i<240; i++ )
    {
        for ( int j=0; j<320; j++ )
        {
            double metric = 100.0 / ( -0.00307 * depth[ ( ( i*2 ) * 640) + ( j*2 ) ] + 3.33 );
            depth_data[i][j] = metric;
        }
    }
    pthread_mutex_unlock(&freenectMutex);
    frame++;
}

    
void *KinectCloud::freenectThreadFunction(void *arg)
{
    KinectCloud *kinectCloud = (KinectCloud*)(arg);


    freenect_set_depth_callback(kinectCloud->f_dev, depthCallback);
    freenect_set_depth_mode(kinectCloud->f_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT));
    
    int res = freenect_start_depth(kinectCloud->f_dev);
    if (res !=0)
    {
        std::cout<<"KinectCloud["<<kinectCloud->getID()<<"] freenect_start_depth failed!"<<std::endl;
    }
    
    //while ( !die && freenect_process_events(kinectCloud->f_ctx) >= 0 )
    while ( freenect_process_events(kinectCloud->f_ctx) >= 0 )
    {
        //usleep(10);
        //freenect_raw_tilt_state* state;
        //state = freenect_get_tilt_state(kinectCloud->f_dev);
        //double dx,dy,dz;
        //freenect_get_mks_accel(state, &dx, &dy, &dz);
        //std::cout<<"\r frame: "<<frame<<" - mks acc: "<<dx<<" "<<dy<<" "<<dz<<"\r";
    }
    
    std::cout<<"KinectCloud["<<kinectCloud->getID()<<"] shutting down streams....";
    
    freenect_stop_depth(kinectCloud->f_dev);
    
    freenect_close_device(kinectCloud->f_dev);
    freenect_shutdown(kinectCloud->f_ctx);
    
    std::cout<<"KinectCloud["<<kinectCloud->getID()<<"] -- done!\n";
    return NULL;
}
    
    
// -----------------------------------------------------------------------------
    
void KinectCloud::setDrawMode (DrawMode mode)
{
	if (this->drawMode_ != (int)mode)
	{
		this->drawMode_ = mode;
		BROADCAST(this, "si", "setDrawMode", getDrawMode());
	}
}

void KinectCloud::setSpacing (float spacing)
{
	if (this->spacing_ != spacing)
	{
		this->spacing_ = spacing;
		BROADCAST(this, "sf", "setSpacing", this->spacing_);
	}
}

void KinectCloud::setRandomCoeff (float randomCoeff)
{
	if (this->randomCoeff_ != randomCoeff)
	{
		this->randomCoeff_ = randomCoeff;
		BROADCAST(this, "sf", "setRandomCoeff", this->randomCoeff_);
	}
}
    
void KinectCloud::setPointSize (float size)
{
    if (this->pointSize_ != size)
    {
        this->pointSize_ = size;
        BROADCAST(this, "sf", "setPointSize", this->pointSize_);
    }
}

void KinectCloud::setColor (float r, float g, float b, float a)
{
	if (this->color_ != osg::Vec4(r,g,b,a))
	{
		this->color_ = osg::Vec4(r,g,b,a);
		BROADCAST(this, "sffff", "setColor", r, g, b, a);
	}
}

// -----------------------------------------------------------------------------
std::vector<lo_message> KinectCloud::getState () const
{
    // inherit state from base class
	std::vector<lo_message> ret = GroupNode::getState();

    lo_message msg;

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

