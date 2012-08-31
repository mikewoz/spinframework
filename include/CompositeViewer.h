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
#include <osg/Timer>

#include "dofppu.h"
#include "ssaoppu.h"
#include "motionblurppu.h"

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

#define VELOCITY_SCALAR 0.004
#define SPIN_SCALAR 0.007

// Subclass of osgPPU::Processor
class PPUProcessor : public osgPPU::Processor
{
    public:
        void onViewportChange()
        {
            ((osgPPU::Processor*)this)->onViewportChange();
            
            osgPPU::Unit* lUnit = NULL;
            // Looking for units which care about the projection matrix
            if((lUnit = findUnit("ssao")) != NULL)
            {
                const osg::Camera* lCamera = getCamera();
                if(lCamera == NULL)
                    return;
                std::cout << "Camera OK" << std::endl;

                /*osgPPU::ShaderAttribute* ssaoShaderAttr = new osgPPU::ShaderAttribute();
                ssaoShaderAttr->add(("vProjectionMatrix"), osg::Uniform::FLOAT_MAT4);
                ssaoShaderAttr->add(("vInvProjectionMatrix"), osg::Uniform::FLOAT_MAT4);

                ssaoShaderAttr->set("vProjectionMatrix", lCamera->getProjectionMatrix());
                ssaoShaderAttr->set("vInvProjectionMatrix", osg::Matrixf::inverse(lCamera->getProjectionMatrix()));

                lUnit->getOrCreateStateSet()->setAttributeAndModes(ssaoShaderAttr);*/
                
                osg::StateSet* lStateSet = lUnit->getOrCreateStateSet();
                if(lStateSet == NULL)
                    return;
                std::cout << "lStateSet OK" << std::endl;

                /*osg::Uniform* lProjMat = lUnit->getOrCreateStateSet()->getUniform("vProjectionMatrix");
                if(lProjMat == NULL)
                    return;
                std::cout << "lProjMat OK" << std::endl; 

                lProjMat->setElement(0, lCamera->getProjectionMatrix());

                osg::Uniform* lInvProjMat = lUnit->getOrCreateStateSet()->getUniform("vInvProjectionMatrix");
                if(lInvProjMat == NULL)
                    return;
                std::cout << "lInvProjMat OK" << std::endl;                

                lInvProjMat->setElement(0, osg::Matrixf::inverse(lCamera->getProjectionMatrix()));*/
            } 
        }
};

#define PPU_NONE        0x0000
#define PPU_DOF         0x0001
#define PPU_SSAO        0x0002
#define PPU_MOTIONBLUR  0x0004

class CompositeViewer : public osgViewer::CompositeViewer
//class CompositeViewer : public osgViewer::Viewer
{
    public:
        // enum of the different PPU effects available
        // Use primes, to allow activation of multiple effects
        enum ppuEffect
        {
            noEffect = 0,
            dofEffect = 1,
            ssaoEffect = 2
        };

        //! Default construcotr
        CompositeViewer(osg::ArgumentParser& args);

        //! Destructor
        ~CompositeViewer();
        
        //! Get the ppu processor
        std::vector<osgPPU::Processor*> getProcessor() { return mProcessors; }

        //! Create camera resulting texture
        osg::Texture* createRenderTexture(int tex_width, int tex_height, bool depth);
        
        //! Setup the camera to do the render to texture
        void setupCamera();
        
        //! Just setup some stuff
        void viewerInit();

        //! Setup osgppu for rendering
        void initializePPU(unsigned int pEffect = noEffect);

        //! Update the frames
        void frame(double f = USE_REFERENCE_TIME);
      
        //! Poll the SpaceNavigator for updates and send velocity/spin 
        void updateSpaceNavigator();
        void setSpaceNavigatorNode(std::string nID) { spnavNodeID_ = nID; }

        /**
         * Scale the velocity effect of navigational devices (for example,
         * scale X and Z axes to zero so that only forward motion is allowed).
	 */
        void setVelocityScalars(osg::Vec3 v) { velocityScalars_ = v; }

         /**
         * Scale the spin effect of navigational devices (for example, disallow
         * roll and allow only pitch and yaw by setting the scale to 1,0,1)
	 */
        void setSpinScalars(osg::Vec3 v) { spinScalars_ = v; }
 
        //int run();

        std::vector<DoFRendering*> mDofPPUs;
        std::vector<SSAORendering*> mSsaoPPUs;
        std::vector<MotionBlurRendering*> mMBlurPPUs;

    private:
        std::vector<osgPPU::Processor*> mProcessors;

        float mOldTime;
        //DoFRendering mDoFSetup;
        bool mbInitialized;
       
	    // navigation update stuff:
        std::string spnavNodeID_; 
        osg::Timer_t lastNavTick_;
	    float speedScaleValue_;
	    float moving_;
	    osg::Vec3 velocityScalars_;
	    osg::Vec3 spinScalars_;

        // Projection matrix from the first rendering,
        // to be used in the PPU
        osg::Matrixd mProjectionMatrix;
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
                bool lIsDof = (viewer->mDofPPUs.size() == viewer->getNumViews());
                bool lIsSSAO = (viewer->mSsaoPPUs.size() == viewer->getNumViews());

                // For each view
                for(unsigned int i=0; i<viewer->getNumViews(); ++i)
                {

                    osgPPU::Camera::resizeViewport(0,0, ea.getWindowWidth(), ea.getWindowHeight(), viewer->getView(i)->getCamera());
            
                    // Get the previous projection matrix information
                    double fovy, aspect, lNear, lFar;
                    viewer->getView(i)->getCamera()->getProjectionMatrixAsPerspective(fovy, aspect, lNear, lFar);
                    // Update the ratio to match the new ratio of the window
                    viewer->getView(i)->getCamera()->setProjectionMatrixAsPerspective(fovy, (float)ea.getWindowWidth()/(float)ea.getWindowHeight(), lNear, lFar);

                    osgPPU::Processor* lProcessor = viewer->getProcessor()[i];
                    lProcessor->onViewportChange();

                    if(lIsDof)
                    {
                        // Reset near and far, although there's no reason for them to have changed
                        viewer->mDofPPUs[i]->setNear(lNear);
                        viewer->mDofPPUs[i]->setFar(lFar);
                    }

                    if(lIsSSAO)
                    {
                        osg::Matrixf lProjMat = viewer->getView(i)->getCamera()->getProjectionMatrix();                    
                        viewer->mSsaoPPUs[i]->setProjectionMatrix(lProjMat);
                    }

                }                
                
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
