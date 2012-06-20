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

#ifndef __CompositeViewer_H
#define __CompositeViewer_H

#include <osgViewer/CompositeViewer>
#include <osgViewer/View>
#include <osgViewer/ViewerEventHandlers>
#include <osg/GraphicsContext>

#include "dofppu.h"

namespace spin
{

struct frustum
{
	bool valid;
	float left;
	float right;
	float bottom;
	float top;
	float near;
	float far;
};



class CompositeViewer : public osgViewer::CompositeViewer
//class CompositeViewer : public osgViewer::Viewer
{
    public:
        //! Default construcotr
        CompositeViewer(osg::ArgumentParser& args);
        
        //! Get the ppu processor
        osgPPU::Processor* getProcessor() { return mProcessor.get(); }

        //! Create camera resulting texture
        osg::Texture* createRenderTexture(int tex_width, int tex_height, bool depth);
        
        //! Setup the camera to do the render to texture
        void setupCamera(osg::Viewport* vp);
        
        //! Handle resizing of the window
        void resize(const int width, const int height);
        
        //! Just setup some stuff
        void viewerInit();

        //! Setup osgppu for rendering
        void initializePPU();

        //! Update the frames
        void frame(double f = USE_REFERENCE_TIME);
        
        //int run();

        osg::ref_ptr<DoFRendering> dofPPU_;

    private:
        osg::ref_ptr<osgPPU::Processor> mProcessor;

        float mOldTime;
        //DoFRendering mDoFSetup;
        bool mbInitialized;
        
        osg::ref_ptr<osg::Texture> colorTexture_;
        osg::ref_ptr<osg::Texture> depthTexture_;
};


class CustomResizeHandler : public osgGA::GUIEventHandler
{
public:
    CompositeViewer* viewer;

    CustomResizeHandler(CompositeViewer* v) : viewer(v)
    {
    }

    bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&)
    {
        switch(ea.getEventType())
        {
            case (osgGA::GUIEventAdapter::RESIZE):
            {
                osgPPU::Camera::resizeViewport(0,0, ea.getWindowWidth(), ea.getWindowHeight(), viewer->getView(0)->getCamera());
                viewer->getProcessor()->onViewportChange();
                break;
            }
            default:
                break;
        }
        return false;
    }
};


osg::Geometry* create3DSphericalDisplayDistortionMesh(const osg::Vec3& origin, const osg::Vec3& widthVector, const osg::Vec3& heightVector, double sphere_radius, double collar_radius, double distance=0.0);

void makeDomeView(osg::GraphicsContext *gc, osg::GraphicsContext::Traits *traits, osg::View *view, osg::Camera *cam, int textureSize, double radius, double collar, double distance, double crop, osg::Image* intensityMap, const osg::Matrixd& projectorMatrix);

void loadXMLcamera(TiXmlElement *XMLnode, osgViewer::Viewer::View *view, osg::Camera *cam, osg::GraphicsContext::Traits *traits, osg::GraphicsContext *gc);

void loadXMLwindow(TiXmlElement *XMLnode, osgViewer::CompositeViewer &viewer);

// this callback is used to update the viewer:
int viewerCallback(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);


} // end of namespace spin


#endif