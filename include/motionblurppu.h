#include <osgPPU/Unit.h>
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

/***************************************/
// PPU setup for motion blur rendering
/***************************************/
class MotionBlurRendering : virtual public osg::Referenced
{
    private:
        osgPPU::ShaderAttribute* motionBlurAttr;

    public:
        /******************/
        MotionBlurRendering()
        {
        }

        /*****************/
        void setMotionBlurFactor(float pFactor)
        {
            if(pFactor >= 0.f && pFactor < 1.f)
                motionBlurAttr->set("vMotionBlurFactor", pFactor);
        }

        /******************/
        void createMotionBlurPipeline(osgPPU::Processor* pParent, osgPPU::Unit*& pLastUnit)
        {
            osg::ref_ptr<osgDB::ReaderWriter::Options> fragmentOptions = new osgDB::ReaderWriter::Options("fragment");            
            osg::ref_ptr<osgDB::ReaderWriter::Options> vertexOptions = new osgDB::ReaderWriter::Options("vertex");

            // If last unit is nullthe first unit will bypass the color output of the camera
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

            // We keep the result of the current rendering for the next pass
            osgPPU::UnitBypass* lPrevious = new osgPPU::UnitBypass();
            lPrevious->setName("previous");

            // Then, motion blur unit
            osgPPU::UnitInOut* lMotionBlur = new osgPPU::UnitInOut();
            lMotionBlur->setName("motionBlur");

            motionBlurAttr = new osgPPU::ShaderAttribute();
            {
                motionBlurAttr->addShader(osgDB::readShaderFile("motionBlur_vp.glsl", vertexOptions.get()));
                motionBlurAttr->addShader(osgDB::readShaderFile("motionBlur_fp.glsl", fragmentOptions.get()));
                motionBlurAttr->setName("motionBlurShader");

                motionBlurAttr->add("vMotionBlurFactor", osg::Uniform::FLOAT);
                motionBlurAttr->set("vMotionBlurFactor", 0.8f);

                lMotionBlur->getOrCreateStateSet()->setAttributeAndModes(motionBlurAttr);
                lMotionBlur->setInputTextureIndexForViewportReference(0);

                lMotionBlur->setInputToUniform(lCurrent, "vCurrentMap", true);
                lMotionBlur->setInputToUniform(lPrevious, "vPreviousMap", true);
            }
 
            lMotionBlur->addChild(lPrevious);

            pLastUnit = lPrevious;
        }
};
