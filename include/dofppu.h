#include <osgPPU/Processor.h>
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


//---------------------------------------------------------------
// PPU setup for DoF Rendering
//
//---------------------------------------------------------------
class DoFRendering : virtual public osg::Referenced
{
    private:
    
        float dofGaussSigma;
        float dofGaussRadius;
        float dofFocalLength;
        float dofFocalRange;
        
        osgPPU::ShaderAttribute* gaussx;
        osgPPU::ShaderAttribute* gaussy;
        osgPPU::ShaderAttribute* dofShaderAttr;
        
    public:

        DoFRendering()
        {
            dofGaussSigma = 1.5f;
            dofGaussRadius = 5.0f;

            dofFocalLength = 0.0f;
            dofFocalRange = 50.0f;

        }
        
        void setGaussSigma(float sigma)
        {
            gaussx->set("sigma", sigma);
            gaussy->set("sigma", sigma);
        }

        void setGaussRadius(float radius)
        {
            gaussx->set("radius", radius);
            gaussy->set("radius", radius);
        }

        void setFocalLength(float f)
        {
            dofShaderAttr->set("focalLength", f);
        }

        void setFocalRange(float range)
        {
            dofShaderAttr->set("focalRange", range);
        }

        void setNear(float near)
        {
            dofShaderAttr->set("zNear", near);
        }

        void setFar(float far)
        {
            dofShaderAttr->set("zFar", far);
        }

        //------------------------------------------------------------------------
        void createDoFPipeline(osgPPU::Processor* parent, osgPPU::Unit*& lastUnit, float zNear, float zFar)
        {
            osg::ref_ptr<osgDB::ReaderWriter::Options> fragmentOptions = new osgDB::ReaderWriter::Options("fragment");
            osg::ref_ptr<osgDB::ReaderWriter::Options> vertexOptions = new osgDB::ReaderWriter::Options("vertex");

            // If last unit is nullthe first unit will bypass the color output of the camera
            osgPPU::Unit* bypass;
            if(lastUnit == NULL)
            {
                bypass = new osgPPU::UnitCameraAttachmentBypass();
                ((osgPPU::UnitCameraAttachmentBypass*)bypass)->setBufferComponent(osg::Camera::COLOR_BUFFER0);
                ((osgPPU::UnitCameraAttachmentBypass*)bypass)->setName("ColorBypass");
                parent->addChild(bypass);
            }
            else
            {
                bypass = lastUnit;
            }

            // next unit will bypass the depth output of the camera
            osgPPU::Unit* depthbypass = new osgPPU::UnitDepthbufferBypass();
            depthbypass->setName("DepthBypass");
            parent->addChild(depthbypass);

            // First we compute the blur value for each pixel
            osgPPU::Unit* dof = new osgPPU::UnitInOut(); 
            dofShaderAttr = new osgPPU::ShaderAttribute();
            {
                osg::Shader* lVShader = osgDB::readShaderFile("dof_vp.glsl", vertexOptions.get());
                osg::Shader* lFShader = osgDB::readShaderFile("dof_computeBlur_fp.glsl", fragmentOptions.get());
                
                dofShaderAttr->addShader(lVShader);
                dofShaderAttr->addShader(lFShader);
                dofShaderAttr->setName("BlurComputeShader");

                dofShaderAttr->add("focalLength", osg::Uniform::FLOAT);
                dofShaderAttr->add("focalRange", osg::Uniform::FLOAT);
                dofShaderAttr->add("zNear", osg::Uniform::FLOAT);
                dofShaderAttr->add("zFar", osg::Uniform::FLOAT);

                dofShaderAttr->set("focalLength", dofFocalLength);
                dofShaderAttr->set("focalRange", dofFocalRange);
                dofShaderAttr->set("zNear", zNear);
                dofShaderAttr->set("zFar", zFar);

                dof->getOrCreateStateSet()->setAttributeAndModes(dofShaderAttr);

                dof->setInputToUniform(depthbypass, "texDepthMap", true);
            }
            depthbypass->addChild(dof);

            gaussx = new osgPPU::ShaderAttribute();
            gaussy = new osgPPU::ShaderAttribute();
            {
                osg::Shader* lVShader = osgDB::readShaderFile("dof_vp.glsl", vertexOptions.get());
                osg::Shader* lFHShader = osgDB::readShaderFile("dof_gauss_1Dx_fp.glsl", fragmentOptions.get());
                osg::Shader* lFVShader = osgDB::readShaderFile("dof_gauss_1Dy_fp.glsl", fragmentOptions.get());

                // Setup horizontal blur shader
                gaussx->addShader(lVShader);
                gaussx->addShader(lFHShader);
                gaussx->setName("BlurHorizontalShader");

                gaussx->add("sigma", osg::Uniform::FLOAT);
                gaussx->add("radius", osg::Uniform::FLOAT);

                gaussx->set("sigma", dofGaussSigma);
                gaussx->set("radius", dofGaussRadius);
                
                // Setup vertical blur shader
                gaussy->addShader(lVShader);
                gaussy->addShader(lFVShader);
                gaussy->setName("BlurVerticalShader");

                gaussy->add("sigma", osg::Uniform::FLOAT);
                gaussy->add("radius", osg::Uniform::FLOAT);
            
                gaussy->set("sigma", dofGaussSigma);
                gaussy->set("radius", dofGaussRadius);
            }

            osgPPU::UnitInOut* blurx = new osgPPU::UnitInOut();
            osgPPU::UnitInOut* blury = new osgPPU::UnitInOut();
            {
                blurx->setName("BlurHorizontal");
                blury->setName("BlurVertical");

                blurx->getOrCreateStateSet()->setAttributeAndModes(gaussx);
                blurx->setInputToUniform(dof, "texBlurMap", true);
                blurx->setInputToUniform(bypass, "texColorMap", true);

                blury->getOrCreateStateSet()->setAttributeAndModes(gaussy);
                blury->setInputToUniform(dof, "texBlurMap", true);
                blury->setInputToUniform(blurx, "texColorMap", true);
            }
            bypass->addChild(blurx);
            blurx->addChild(blury);

            lastUnit = blury;
            //lastUnit = dof;
        }
};

