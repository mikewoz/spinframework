#include <osgPPU/Unit.h>
#include <osgPPU/UnitCamera.h>
#include <osgPPU/UnitInOut.h>
#include <osgPPU/UnitText.h>
#include <osgPPU/UnitInResampleOut.h>
#include <osgPPU/UnitInMipmapOut.h>
#include <osgPPU/UnitOut.h>
#include <osgPPU/UnitOutCapture.h>
#include <osgPPU/UnitBypass.h>
#include <osgPPU/UnitTexture.h>
#include <osgPPU/UnitDepthbufferBypass.h>
#include <osgDB/ReaderWriter>
#include <osgDB/ReadFile>
#include <osgPPU/ShaderAttribute.h>

/**********************************************************/
// This PPU applies a mask created from a specified camera
// to the camera linked to the current pipeline
/**********************************************************/
class MaskRendering : virtual public osg::Referenced
{
    private:
        osgPPU::ShaderAttribute* maskAttr;
        osgPPU::ShaderAttribute* maskLightAttr;

    public:
        /************/
        MaskRendering()
        {
        }

        /***********/
        void setMaskTransparency(float pTransparency)
        {
            if(pTransparency >= 0.f && pTransparency <= 1.f)
                maskAttr->set("uTransparency", pTransparency);
        }

        /***********/
        void setMaskLightingDistance(float pDist)
        {
            if(pDist >= 0.f)
            {
                maskAttr->set("uLightingDistance", pDist);
                maskLightAttr->set("uLightingDistance", pDist);
            }
        }

        /***********/
        void setLightSearchDistance(float pDist)
        {
            if(pDist >= 0.f)
            {
                maskLightAttr->set("uMaxLightSearch", pDist);
            }
        }

        /***********/
        void setLightSearchStep(float pStep)
        {
            if(pStep >= 1.f)
            {
                maskLightAttr->set("uLightSearchStep", pStep);
            }
        }

        /***********/
        void createMaskPipeline(osgPPU::Processor* pParent, osgPPU::Unit*& pLastUnit, osg::Camera* pCamera)
        {
            osg::ref_ptr<osgDB::ReaderWriter::Options> fragmentOptions = new osgDB::ReaderWriter::Options("fragment");
            osg::ref_ptr<osgDB::ReaderWriter::Options> vertexOptions = new osgDB::ReaderWriter::Options("vertex");

            double left,right,bottom,top,near,far;
            pParent->getCamera()->getProjectionMatrixAsFrustum(left, right, bottom, top, near, far);

            // If last unit is null the first unit will bypass the color output of the camera,
            // as well as the depth output
            osgPPU::Unit* lColorBypass;
            if(pLastUnit == NULL)
            {
                lColorBypass = new osgPPU::UnitCameraAttachmentBypass();
                ((osgPPU::UnitCameraAttachmentBypass*)lColorBypass)->setBufferComponent(osg::Camera::COLOR_BUFFER0);
                ((osgPPU::UnitCameraAttachmentBypass*)lColorBypass)->setName("current");
                pParent->addChild(lColorBypass);
            }
            else
            {
                lColorBypass = pLastUnit;
            }

            osgPPU::Unit* lDepthBypass = new osgPPU::UnitDepthbufferBypass();
            lDepthBypass->setName("depthBypass");
            pParent->addChild(lDepthBypass);

            // Create a unit for the additional camera
            osgPPU::UnitCamera* lMaskCamera = new osgPPU::UnitCamera();
            {
                lMaskCamera->setCamera(pCamera);
                lMaskCamera->setName("maskCamera");
            }
            pParent->addChild(lMaskCamera);

            // Create the bypass for the first color buffer of this camera
            osgPPU::UnitCameraAttachmentBypass* lMaskBypass;
            osgPPU::UnitCameraAttachmentBypass* lMaskDepth;
            {
                lMaskBypass = new osgPPU::UnitCameraAttachmentBypass();
                lMaskBypass->setBufferComponent(osg::Camera::COLOR_BUFFER0);
                lMaskBypass->setName("maskBypass");
                
                lMaskDepth = new osgPPU::UnitCameraAttachmentBypass();
                lMaskDepth->setBufferComponent(osg::Camera::DEPTH_BUFFER);
                lMaskDepth->setName("maskDepth");
            }
            lMaskCamera->addChild(lMaskBypass);
            lMaskCamera->addChild(lMaskDepth);

            // Compute the mask to get the max lighting distance
            // We do this in lower res to keep resource usage low
            osgPPU::UnitInResampleOut* lMaskResample = new osgPPU::UnitInResampleOut();
            osgPPU::UnitInResampleOut* lMaskDepthResample = new osgPPU::UnitInResampleOut();
            {
                lMaskResample->setName("maskResample");
                lMaskResample->setFactorX(0.5f);
                lMaskResample->setFactorY(0.5f);

                lMaskDepthResample->setName("maskResample");
                lMaskDepthResample->setFactorX(0.5f);
                lMaskDepthResample->setFactorY(0.5f);
            }
            lMaskBypass->addChild(lMaskResample);
            lMaskDepth->addChild(lMaskDepthResample);

            osgPPU::Unit* lMaskLight = new osgPPU::UnitInOut();
            maskLightAttr = new osgPPU::ShaderAttribute();
            {
                osg::Shader* lVShader = osgDB::readShaderFile("mask_vp.glsl", vertexOptions.get());
                osg::Shader* lFShader = osgDB::readShaderFile("mask_fp.glsl", fragmentOptions.get());

                maskLightAttr->addShader(lVShader);
                maskLightAttr->addShader(lFShader);
                maskLightAttr->setName("maskLightShader");

                maskLightAttr->add("uPass", osg::Uniform::INT);
                maskLightAttr->add("uMaxLightSearch", osg::Uniform::FLOAT);
                maskLightAttr->add("uLightSearchStep", osg::Uniform::FLOAT);
                maskLightAttr->add("uLightingDistance", osg::Uniform::FLOAT);
                maskLightAttr->add("uNear", osg::Uniform::FLOAT);
                maskLightAttr->add("uFar", osg::Uniform::FLOAT);
                maskLightAttr->add("uLeft", osg::Uniform::FLOAT);
                maskLightAttr->add("uRight", osg::Uniform::FLOAT);
                maskLightAttr->add("uTop", osg::Uniform::FLOAT);
                maskLightAttr->add("uBottom", osg::Uniform::FLOAT);

                maskLightAttr->set("uPass", 1);
                maskLightAttr->set("uMaxLightSearch", 5.f);
                maskLightAttr->set("uLightSearchStep", 1.f);
                maskLightAttr->set("uLightingDistance", 0.f);
                maskLightAttr->set("uNear", (float)near);
                maskLightAttr->set("uFar", (float)far);
                maskLightAttr->set("uLeft", (float)left);
                maskLightAttr->set("uRight", (float)right);
                maskLightAttr->set("uTop", (float)top);
                maskLightAttr->set("uBottom", (float)bottom);

                lMaskLight->getOrCreateStateSet()->setAttributeAndModes(maskLightAttr);
                lMaskLight->setInputToUniform(lMaskResample, "uMaskMap", true);
                lMaskLight->setInputToUniform(lMaskDepthResample, "uMaskDepthMap", true);
            }

            // We need to resample the result to the correct resolution
            osgPPU::UnitInResampleOut* lMaskBlurredDepth = new osgPPU::UnitInResampleOut();
            {
                lMaskBlurredDepth->setName("maskBlurredDepth");
                lMaskBlurredDepth->setFactorX(2.f);
                lMaskBlurredDepth->setFactorY(2.f);
            }
            lMaskLight->addChild(lMaskBlurredDepth);

            // Apply the mask
            osgPPU::Unit* lMask = new osgPPU::UnitInOut();
            lMask->setName("maskInOut");
            maskAttr = new osgPPU::ShaderAttribute();
            {
                osg::Shader* lVShader = osgDB::readShaderFile("mask_vp.glsl", vertexOptions.get());
                osg::Shader* lFShader = osgDB::readShaderFile("mask_fp.glsl", fragmentOptions.get());

                maskAttr->addShader(lVShader);
                maskAttr->addShader(lFShader);
                maskAttr->setName("maskShader");

                maskAttr->add("uPass", osg::Uniform::INT);
                maskAttr->add("uTransparency", osg::Uniform::FLOAT);
                maskAttr->add("uLightingDistance", osg::Uniform::FLOAT);
                maskAttr->add("uNear", osg::Uniform::FLOAT);
                maskAttr->add("uFar", osg::Uniform::FLOAT);

                maskAttr->set("uPass", 2);
                maskAttr->set("uTransparency", 0.5f);
                maskAttr->set("uLightingDistance", 0.f);
                maskAttr->set("uNear", (float)near);
                maskAttr->set("uFar", (float)far);
                
                lMask->getOrCreateStateSet()->setAttributeAndModes(maskAttr);
                lMask->setInputToUniform(lColorBypass, "uColorMap", true);
                lMask->setInputToUniform(lDepthBypass, "uDepthMap", true);
                lMask->setInputToUniform(lMaskBypass, "uMaskMap", true);
                lMask->setInputToUniform(lMaskBlurredDepth, "uMaskDepthMap", true);
            }

            pLastUnit = lMask;
        }
};
