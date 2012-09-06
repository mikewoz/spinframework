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

            // If last unit is null the first unit will bypass the color output of the camera
            osgPPU::Unit* lCurrent;
            if(pLastUnit == NULL)
            {
                lCurrent = new osgPPU::UnitCameraAttachmentBypass();
                ((osgPPU::UnitCameraAttachmentBypass*)lCurrent)->setBufferComponent(osg::Camera::COLOR_BUFFER0);
                ((osgPPU::UnitCameraAttachmentBypass*)lCurrent)->setName("current");
                pParent->addChild(lCurrent);
            }
            else
            {
                lCurrent = pLastUnit;
            }

            // Get the bypass from the pipeline's camera
            osgPPU::UnitCameraAttachmentBypass* lColorBypass;
            {
                lColorBypass = new osgPPU::UnitCameraAttachmentBypass();
                lColorBypass->setBufferComponent(osg::Camera::COLOR_BUFFER0);
                lColorBypass->setName("colorBypass");
            }
            lCurrent->addChild(lColorBypass);

            // Create a unit for the additional camera
            osgPPU::UnitCamera* lMaskCamera = new osgPPU::UnitCamera();
            {
                lMaskCamera->setCamera(pCamera);
                lMaskCamera->setName("maskCamera");
            }
            
            // Create the bypass for the first color buffer of this camera
            osgPPU::UnitCameraAttachmentBypass* lMaskBypass;
            {
                lMaskBypass = new osgPPU::UnitCameraAttachmentBypass();
                lMaskBypass->setBufferComponent(osg::Camera::COLOR_BUFFER0);
                lMaskBypass->setName("maskBypass");
            }
            lMaskCamera->addChild(lMaskBypass);

            // Apply the mask
            osgPPU::Unit* lMask = new osgPPU::UnitInOut();
            maskAttr = new osgPPU::ShaderAttribute();
            {
                osg::Shader* lVShader = osgDB::readShaderFile("mask_vp.glsl", vertexOptions.get());
                osg::Shader* lFShader = osgDB::readShaderFile("mask_fp.glsl", fragmentOptions.get());

                maskAttr->addShader(lVShader);
                maskAttr->addShader(lFShader);
                maskAttr->setName("maskShader");

                lMask->getOrCreateStateSet()->setAttributeAndModes(maskAttr);
                lMask->setInputToUniform(lColorBypass, "uColorMap", true);
                lMask->setInputToUniform(lMaskBypass, "uMaskMap", true);
            }

            pLastUnit = lMask;
        }
};
