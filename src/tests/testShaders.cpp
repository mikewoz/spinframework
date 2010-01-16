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

#include <string>
#include <iostream>
#include <sstream>

#include <osgViewer/Viewer>
#include <osgViewer/View>
#include <osgViewer/ViewerEventHandlers>

#include <osgGA/NodeTrackerManipulator>
#include <osgGA/TrackballManipulator>
#include <osgGA/StateSetManipulator>

#include <osg/Texture3D>
#include <osgDB/ReadFile>
#include <osgDB/FileUtils>

#include <osgDB/ReadFile>

#include <osg/Program>
#include <osg/Shader>
#include <osg/Uniform>

#include <osg/io_utils>

#include "Noise.h"
#include "spinUtil.h"
#include "spinContext.h"
#include "osgUtil.h"
#include "ShapeNode.h"

using namespace std;

extern pthread_mutex_t pthreadLock;


// *****************************************************************************
// *****************************************************************************
// *****************************************************************************

// load source from a file.
static void
LoadShaderSource( osg::Shader* shader, const std::string& fileName )
{
    std::string fqFileName = osgDB::findDataFile(fileName);
    if( fqFileName.length() != 0 )
    {
        shader->loadShaderSourceFromFile( fqFileName.c_str() );
    }
    else
    {
        osg::notify(osg::WARN) << "File \"" << fileName << "\" not found." << std::endl;
    }
}


///////////////////////////////////////////////////////////////////////////
// for demo simplicity, this one callback animates all the shaders, instancing
// for each uniform but with a specific operation each time.

class AnimateCallback: public osg::Uniform::Callback
{
    public:
    
        enum Operation
        {
            OFFSET,
            SIN,
            COLOR1,
            COLOR2            
        };
    
        AnimateCallback(Operation op) : _enabled(true),_operation(op) {}

        virtual void operator() ( osg::Uniform* uniform, osg::NodeVisitor* nv )
        {
            if( _enabled )
            {
                float angle = 2.0 * nv->getFrameStamp()->getSimulationTime();
                float sine = sinf( angle );        // -1 -> 1
                float v01 = 0.5f * sine + 0.5f;        //  0 -> 1
                float v10 = 1.0f - v01;                //  1 -> 0
                switch(_operation)
                {
                    case OFFSET : uniform->set( osg::Vec3(0.505f, 0.8f*v01, 0.0f) ); break;
                    case SIN : uniform->set( sine ); break;
                    case COLOR1 : uniform->set( osg::Vec3(v10, 0.0f, 0.0f) ); break;
                    case COLOR2 : uniform->set( osg::Vec3(v01, v01, v10) ); break;
                }
            }
        }

    private:
        bool _enabled;
        Operation _operation;
};


osg::ref_ptr<osg::Image> image;

static osg::Image*
make3DNoiseImage(int texSize)
{
    image = new osg::Image;
    image->setImage(texSize, texSize, texSize,
            4, GL_RGBA, GL_UNSIGNED_BYTE,
            new unsigned char[4 * texSize * texSize * texSize],
            osg::Image::USE_NEW_DELETE);

    const int startFrequency = 4;
    const int numOctaves = 4;

    int f, i, j, k, inc;
    double ni[3];
    double inci, incj, inck;
    int frequency = startFrequency;
    GLubyte *ptr;
    double amp = 0.5;

    osg::notify(osg::INFO) << "creating 3D noise texture... ";

    for (f = 0, inc = 0; f < numOctaves; ++f, frequency *= 2, ++inc, amp *= 0.5)
    {
        SetNoiseFrequency(frequency);
        ptr = image->data();
        ni[0] = ni[1] = ni[2] = 0;

        inci = 1.0 / (texSize / frequency);
        for (i = 0; i < texSize; ++i, ni[0] += inci)
        {
            incj = 1.0 / (texSize / frequency);
            for (j = 0; j < texSize; ++j, ni[1] += incj)
            {
                inck = 1.0 / (texSize / frequency);
                for (k = 0; k < texSize; ++k, ni[2] += inck, ptr += 4)
                {
                    *(ptr+inc) = (GLubyte) (((noise3(ni) + 1.0) * amp) * 128.0);
                }
            }
        }
    }

    osg::notify(osg::INFO) << "DONE" << std::endl;
    return image.get();        
}

static osg::Texture3D*
make3DNoiseTexture(int texSize )
{
    osg::Texture3D* noiseTexture = new osg::Texture3D;
    noiseTexture->setFilter(osg::Texture3D::MIN_FILTER, osg::Texture3D::LINEAR);
    noiseTexture->setFilter(osg::Texture3D::MAG_FILTER, osg::Texture3D::LINEAR);
    noiseTexture->setWrap(osg::Texture3D::WRAP_S, osg::Texture3D::REPEAT);
    noiseTexture->setWrap(osg::Texture3D::WRAP_T, osg::Texture3D::REPEAT);
    noiseTexture->setWrap(osg::Texture3D::WRAP_R, osg::Texture3D::REPEAT);
    noiseTexture->setImage( make3DNoiseImage(texSize) );
    return noiseTexture;
}


#define TEXUNIT_NOISE 2








// class to handle events with a pick
class PickHandler : public osgGA::GUIEventHandler {
public: 

    PickHandler(spinContext* s): spin(s) {}
    ~PickHandler() {}
    
    bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa);

    virtual void pick(osgViewer::View* view, const osgGA::GUIEventAdapter& ea);

protected:

	spinContext *spin;
};

bool PickHandler::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa)
{
    switch(ea.getEventType())
    {
        case(osgGA::GUIEventAdapter::PUSH):
        {
            osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
            if (view) pick(view,ea);
            return false;
        }
        /*
        case(osgGA::GUIEventAdapter::KEYDOWN):
        {
            if (ea.getKey()=='c')
            {        
                osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
                osg::ref_ptr<osgGA::GUIEventAdapter> event = new osgGA::GUIEventAdapter(ea);
                event->setX((ea.getXmin()+ea.getXmax())*0.5);
                event->setY((ea.getYmin()+ea.getYmax())*0.5);
                if (view) pick(view,*event);
            }
            return false;
        }
        */
        default:
            return false;
    }
}

void PickHandler::pick(osgViewer::View* view, const osgGA::GUIEventAdapter& ea)
{
    osgUtil::LineSegmentIntersector::Intersections intersections;

    std::string gdlist="";
    float x = ea.getX();
    float y = ea.getY();
#if 0
    osg::ref_ptr< osgUtil::LineSegmentIntersector > picker = new osgUtil::LineSegmentIntersector(osgUtil::Intersector::WINDOW, x, y);
    osgUtil::IntersectionVisitor iv(picker.get());
    view->getCamera()->accept(iv);
    if (picker->containsIntersections())
    {
        intersections = picker->getIntersections();
#else
    if (view->computeIntersections(x,y,intersections))
    {
#endif
    	osgUtil::LineSegmentIntersector::Intersections::iterator hitr;
        for(hitr = intersections.begin(); hitr != intersections.end(); ++hitr)
        {
            std::ostringstream os;
            if (!hitr->nodePath.empty() && !(hitr->nodePath.back()->getName().empty()))
            {
                // the geodes are identified by name.
                os<<"Object \""<<hitr->nodePath.back()->getName()<<"\""<<std::endl;
            }
            else if (hitr->drawable.valid())
            {
                os<<"Object \""<<hitr->drawable->className()<<"\""<<std::endl;
            }

            os<<"        local coords vertex("<< hitr->getLocalIntersectPoint()<<")"<<"  normal("<<hitr->getLocalIntersectNormal()<<")"<<std::endl;
            os<<"        world coords vertex("<< hitr->getWorldIntersectPoint()<<")"<<"  normal("<<hitr->getWorldIntersectNormal()<<")"<<std::endl;
            const osgUtil::LineSegmentIntersector::Intersection::IndexList& vil = hitr->indexList;
            for(unsigned int i=0;i<vil.size();++i)
            {
                os<<"        vertex indices ["<<i<<"] = "<<vil[i]<<std::endl;
            }
            
            gdlist += os.str();
        }
        std::cout << gdlist << std::endl;
        
        // just take first hit:
        if (intersections.size())
        {
        	osg::Vec3 hitPoint = intersections.begin()->getLocalIntersectPoint();
        	float scale = 1.0f;
        	int _row = (int) ( (hitPoint.z()+(scale/2)) * image->t() );
        	int _col = (int) ( (hitPoint.x()+(scale/2)) * image->s() );
        	
        	std::cout << "drawing on (" << image->s() << "x" << image->t() << ") image: row=" << _row << ", col=" << _col << std::endl;
        	
        	unsigned char* pixel = image->data( _row, _col );
        	*(pixel) = (GLubyte) 0;
        	image->dirty(); // force update next frame
            	
        }
    }
    
}




osg::Camera* createHUD()
{
    // create a camera to set up the projection and model view matrices, and the subgraph to drawn in the HUD
    osg::Camera* camera = new osg::Camera;

    // set the projection matrix
    camera->setProjectionMatrix(osg::Matrix::ortho2D(0,1280,0,1024));

    // set the view matrix    
    camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    camera->setViewMatrix(osg::Matrix::identity());

    // only clear the depth buffer
    camera->setClearMask(GL_DEPTH_BUFFER_BIT);

    // draw subgraph after main camera view.
    camera->setRenderOrder(osg::Camera::POST_RENDER);

    // we don't want the camera to grab event focus from the viewers main camera(s).
    camera->setAllowEventFocus(false);
   

    // add to this camera a subgraph to render
    {

        osg::Geode* geode = new osg::Geode();

        std::string timesFont("fonts/arial.ttf");

        // turn lighting off for the text and disable depth test to ensure its always ontop.
        osg::StateSet* stateset = geode->getOrCreateStateSet();
        stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

        osg::Vec3 position(150.0f,800.0f,0.0f);
        osg::Vec3 delta(0.0f,-120.0f,0.0f);

        {
            osgText::Text* text = new  osgText::Text;
            geode->addDrawable( text );

            text->setFont(timesFont);
            text->setPosition(position);
            text->setText("Head Up Displays are simple :-)");

            position += delta;
        }    


        {
            osgText::Text* text = new  osgText::Text;
            geode->addDrawable( text );

            text->setFont(timesFont);
            text->setPosition(position);
            text->setText("All you need to do is create your text in a subgraph.");

            position += delta;
        }    


        {
            osgText::Text* text = new  osgText::Text;
            geode->addDrawable( text );

            text->setFont(timesFont);
            text->setPosition(position);
            text->setText("Then place an osg::Camera above the subgraph\n"
                          "to create an orthographic projection.\n");

            position += delta;
        } 

        {
            osgText::Text* text = new  osgText::Text;
            geode->addDrawable( text );

            text->setFont(timesFont);
            text->setPosition(position);
            text->setText("Set the Camera's ReferenceFrame to ABSOLUTE_RF to ensure\n"
                          "it remains independent from any external model view matrices.");

            position += delta;
        } 

        {
            osgText::Text* text = new  osgText::Text;
            geode->addDrawable( text );

            text->setFont(timesFont);
            text->setPosition(position);
            text->setText("And set the Camera's clear mask to just clear the depth buffer.");

            position += delta;
        }    

        {
            osgText::Text* text = new  osgText::Text;
            geode->addDrawable( text );

            text->setFont(timesFont);
            text->setPosition(position);
            text->setText("And finally set the Camera's RenderOrder to POST_RENDER\n"
                          "to make sure its drawn last.");

            position += delta;
        }    


        {
            osg::BoundingBox bb;
            for(unsigned int i=0;i<geode->getNumDrawables();++i)
            {
                bb.expandBy(geode->getDrawable(i)->getBound());
            }

            osg::Geometry* geom = new osg::Geometry;

            osg::Vec3Array* vertices = new osg::Vec3Array;
            float depth = bb.zMin()-0.1;
            vertices->push_back(osg::Vec3(bb.xMin(),bb.yMax(),depth));
            vertices->push_back(osg::Vec3(bb.xMin(),bb.yMin(),depth));
            vertices->push_back(osg::Vec3(bb.xMax(),bb.yMin(),depth));
            vertices->push_back(osg::Vec3(bb.xMax(),bb.yMax(),depth));
            geom->setVertexArray(vertices);

            osg::Vec3Array* normals = new osg::Vec3Array;
            normals->push_back(osg::Vec3(0.0f,0.0f,1.0f));
            geom->setNormalArray(normals);
            geom->setNormalBinding(osg::Geometry::BIND_OVERALL);

            osg::Vec4Array* colors = new osg::Vec4Array;
            colors->push_back(osg::Vec4(1.0f,1.0,0.8f,0.2f));
            geom->setColorArray(colors);
            geom->setColorBinding(osg::Geometry::BIND_OVERALL);

            geom->addPrimitiveSet(new osg::DrawArrays(GL_QUADS,0,4));

            osg::StateSet* stateset = geom->getOrCreateStateSet();
            stateset->setMode(GL_BLEND,osg::StateAttribute::ON);
            //stateset->setAttribute(new osg::PolygonOffset(1.0f,1.0f),osg::StateAttribute::ON);
            stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

            geode->addDrawable(geom);
        }

        camera->addChild(geode);
    }

    return camera;
}



// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
int main(int argc, char **argv)
{

	spinContext *spin = new spinContext(spinContext::LISTENER_MODE);

	// *************************************************************************

	// get arguments:
	osg::ArgumentParser arguments(&argc,argv);

	// For testing purposes, we allow loading a scene with a commandline arg:
	osg::ref_ptr<osg::Node> argScene = osgDB::readNodeFiles(arguments);

	// *************************************************************************
	// construct the viewer:
	osgViewer::Viewer viewer = osgViewer::Viewer(arguments);
	viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);

	// *************************************************************************
	// any option left unread are converted into errors to write out later.
	arguments.reportRemainingOptionsAsUnrecognized();

	// report any errors if they have occured when parsing the program aguments.
	if (arguments.errors())
	{
		arguments.writeErrorMessages(std::cout);
		return 1;
	}
	
	// *************************************************************************
	// start the listener thread:
	if (!spin->start())
	{
        std::cout << "ERROR: could not start SPIN listener thread" << std::endl;
        exit(1);
	}

	spin->sceneManager->setGraphical(true);		

	// *************************************************************************
	// set up any initial scene elements:
	if (argScene.valid()) {
		std::cout << "Loading sample model" << std::endl;
		spin->sceneManager->worldNode->addChild(argScene.get());
	}


	// *************************************************************************
	// viewer stuff:
	viewer.getUsage(*arguments.getApplicationUsage());
	viewer.setSceneData(spin->sceneManager->rootNode.get());
	viewer.setUpViewInWindow(50,50,320,240);
	viewer.setCameraManipulator(new osgGA::TrackballManipulator());

	
	osgViewer::Viewer::Windows windows;
	viewer.getWindows(windows);
	
	if (windows.empty()) return 1;

	osg::Camera* hudCamera = createHUD();

	// set up cameras to rendering on the first window available.
	hudCamera->setViewport(0,0,windows[0]->getTraits()->width, windows[0]->getTraits()->height);
	viewer.addSlave(hudCamera, false);
	
	
	
	
	// *************************************************************************
	// set up picker:
	viewer.addEventHandler(new PickHandler(spin));
	
	
	// *************************************************************************
	// shader stuff:

	osg::ref_ptr<ShapeNode> shp = dynamic_cast<ShapeNode*>(spin->sceneManager->createNode("shp", "ShapeNode"));
	shp->setShape(ShapeNode::PLANE);
	shp->setTextureFromFile("/Users/mikewoz/svn/ss_demo/images/abstract03.jpg");
	
	if (!shp->shapeGeode.valid())
	{
		std::cout << "bad shapeGeode" << std::endl;
		exit(1);
	}
	
	osg::Program* ErodedProgram = new osg::Program;
    osg::Shader* ErodedVertObj = new osg::Shader( osg::Shader::VERTEX );
    osg::Shader* ErodedFragObj = new osg::Shader( osg::Shader::FRAGMENT );
    
    ErodedProgram->setName( "eroded" );
    ErodedProgram->addShader( ErodedFragObj );
    ErodedProgram->addShader( ErodedVertObj );
	

	osg::StateSet *ss = shp->shapeGeode->getOrCreateStateSet();
	

    
	osg::Texture3D* noiseTexture = make3DNoiseTexture( 32 ); // 128
    ss->setTextureAttribute(TEXUNIT_NOISE, noiseTexture);
    ss->setAttributeAndModes(ErodedProgram, osg::StateAttribute::ON);


    ss->addUniform( new osg::Uniform("LightPosition", osg::Vec3(0.0f, 0.0f, 4.0f)) );
    ss->addUniform( new osg::Uniform("Scale", 1.0f) );
    ss->addUniform( new osg::Uniform("sampler3d", TEXUNIT_NOISE) );
    
    
    {
    	/*
        osg::Uniform* OffsetUniform = new osg::Uniform( "Offset", osg::Vec3(0.0f, 0.0f, 0.0f) );
        OffsetUniform->setUpdateCallback(new AnimateCallback(AnimateCallback::OFFSET));
        ss->addUniform( OffsetUniform );
        
        osg::Uniform* SineUniform   = new osg::Uniform( "Sine", 0.0f );
        SineUniform->setUpdateCallback(new AnimateCallback(AnimateCallback::SIN));
        ss->addUniform( SineUniform );
        
        osg::Uniform* Color1Uniform = new osg::Uniform( "Color1", osg::Vec3(0.0f, 0.0f, 0.0f) );
        Color1Uniform->setUpdateCallback(new AnimateCallback(AnimateCallback::COLOR1));
        ss->addUniform( Color1Uniform );
        
        osg::Uniform* Color2Uniform = new osg::Uniform( "Color2", osg::Vec3(0.0f, 0.0f, 0.0f) );
        Color2Uniform->setUpdateCallback(new AnimateCallback(AnimateCallback::COLOR2));
        ss->addUniform( Color2Uniform );
        */
    }
	

    LoadShaderSource( ErodedVertObj, "../shaders/eroded.vert" );
    LoadShaderSource( ErodedFragObj, "../shaders/eroded.frag" );
	
	
    //LoadShaderSource( ErodedVertObj, "post_glowBalloon.vert" );
    //LoadShaderSource( ErodedFragObj, "post_glowBalloon.frag" );
	
	
	
	
	
	
	// *************************************************************************
	// start threads:
	viewer.realize();

	// program loop:
	while( !viewer.done() && spin->isRunning() )
	{
		pthread_mutex_lock(&pthreadLock);
		spin->sceneManager->updateGraph();
		pthread_mutex_unlock(&pthreadLock);

		pthread_mutex_lock(&pthreadLock);
		viewer.frame();
		pthread_mutex_unlock(&pthreadLock);
	}

	return 0;
}
