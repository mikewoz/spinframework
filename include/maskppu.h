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

    public:
        /************/
        MaskRendering()
        {
        }

        /***********/
        void createMaskPipeline(osgPPU::Processor* pParent, osgPPU::Unit*& pLastUnit, osg::Camera* pCamera)
        {
            osg::ref_ptr<osgDB::ReaderWriter::Options> fragmentOptions = new osgDB::ReaderWriter::Options("fragment");
            osg::ref_ptr<osgDB::ReaderWriter::Options> vertexOptions = new osgDB::ReaderWriter::Options("vertex");

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

            osgPPU::UnitDepthbufferBypass* lDepthBypass = new osgPPU::UnitDepthbufferBypass();
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

                lMask->getOrCreateStateSet()->setAttributeAndModes(maskAttr);
                lMask->setInputToUniform(lColorBypass, "uColorMap", true);
                lMask->setInputToUniform(lDepthBypass, "uDepthMap", true);
                lMask->setInputToUniform(lMaskBypass, "uMaskMap", true);
                lMask->setInputToUniform(lMaskDepth, "uMaskDepthMap", true);
            }

            pLastUnit = lMask;
        }
};
