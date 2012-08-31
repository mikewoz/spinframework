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
        osgPPU::ShaderAttribute* blurAttr;

    public:
        /******************/
        MotionBlurRendering()
        {
        }

        /*****************/
        void setMotionBlurFactor(float pFactor)
        {
            if(pFactor >= 0.f && pFactor < 1.f)
            {
                motionBlurAttr->set("vMotionBlurFactor", pFactor);
                blurAttr->set("vMotionBlurFactor", pFactor);
            }
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

            // We also get the motion blur values for each pixel
            // This also contains the flag for activation of motion blur with specific value,
            // or usage of the global value (vMotionBlurFactor)
            osgPPU::Unit* lMotionFactor;
            {
                lMotionFactor = new osgPPU::UnitCameraAttachmentBypass();
                ((osgPPU::UnitCameraAttachmentBypass*)lMotionFactor)->setBufferComponent(osg::Camera::COLOR_BUFFER3);
                ((osgPPU::UnitCameraAttachmentBypass*)lMotionFactor)->setName("motionFactor");
                pParent->addChild(lMotionFactor);
            }

            // We keep the result of the current rendering for the next pass
            osgPPU::UnitBypass* lPreviousColor = new osgPPU::UnitBypass();
            lPreviousColor->setName("previousColor");
            
            osgPPU::UnitBypass* lPreviousBlur = new osgPPU::UnitBypass();
            lPreviousBlur->setName("previousBlur");

            // The following is needed to blur behind objects, when background doesn't have the same
            // motion blur factor
            osgPPU::UnitInOut* lBlurBlur = new osgPPU::UnitInOut();
            lBlurBlur->setName("blurBlur");
            blurAttr = new osgPPU::ShaderAttribute();
            {
                blurAttr->addShader(osgDB::readShaderFile("motionBlur_vp.glsl", vertexOptions.get()));
                blurAttr->addShader(osgDB::readShaderFile("motionBlur_fp.glsl", fragmentOptions.get()));
                blurAttr->setName("blurBlurAttr");

                blurAttr->add("vMotionBlurFactor", osg::Uniform::FLOAT);
                blurAttr->set("vMotionBlurFactor", 0.8f);

                blurAttr->add("vPass", osg::Uniform::INT);
                blurAttr->set("vPass", 1);

                lBlurBlur->getOrCreateStateSet()->setAttributeAndModes(blurAttr);
                lBlurBlur->setInputTextureIndexForViewportReference(0);

                lBlurBlur->setInputToUniform(lMotionFactor, "vFactorMap", true);
                lBlurBlur->setInputToUniform(lPreviousBlur, "vPreviousMap", true);
            }

            lBlurBlur->addChild(lPreviousBlur);

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

                motionBlurAttr->add("vPass", osg::Uniform::INT);
                motionBlurAttr->set("vPass", 2);

                lMotionBlur->getOrCreateStateSet()->setAttributeAndModes(motionBlurAttr);
                lMotionBlur->setInputTextureIndexForViewportReference(0);

                lMotionBlur->setInputToUniform(lBlurBlur, "vFactorMap", true);
                lMotionBlur->setInputToUniform(lCurrent, "vCurrentMap", true);
                // This last uniform MUST be last, otherwise the loop we are trying to create with lPreviousColor
                // won't work. Bug in osgPPU ?
                lMotionBlur->setInputToUniform(lPreviousColor, "vPreviousMap", true);
            }
 
            lMotionBlur->addChild(lPreviousColor);

            pLastUnit = lMotionBlur;
        }
};
