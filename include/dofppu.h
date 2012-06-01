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

            dofFocalLength = 15.0f;
            dofFocalRange = 13.0f;

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

            // the first unit will bypass the color output of the camera
            osgPPU::UnitBypass* bypass = new osgPPU::UnitBypass();
            bypass->setName("ColorBypass");
            parent->addChild(bypass);

            // next unit will bypass the depth output of the camera
            osgPPU::UnitDepthbufferBypass* depthbypass = new osgPPU::UnitDepthbufferBypass();
            depthbypass->setName("DepthBypass");
            parent->addChild(depthbypass);

            // we need to blur the output of the color texture to emulate
            // the depth of field. Therefor first we just resample
            osgPPU::UnitInResampleOut* resampleLight = new osgPPU::UnitInResampleOut();
            {
                resampleLight->setName("ResampleLight");
                resampleLight->setFactorX(0.5);
                resampleLight->setFactorY(0.5);
            }
            bypass->addChild(resampleLight);


            // helper shader class to perform gauss blur
            //osgPPU::ShaderAttribute* gaussx = new osgPPU::ShaderAttribute();
            //osgPPU::ShaderAttribute* gaussy = new osgPPU::ShaderAttribute();
            gaussx = new osgPPU::ShaderAttribute();
            gaussy = new osgPPU::ShaderAttribute();
            {
                // read shaders from file
                osg::Shader* vshader = osgDB::readShaderFile("gauss_convolution_vp.glsl", vertexOptions.get());
                osg::Shader* fhshader = osgDB::readShaderFile("gauss_convolution_1Dx_fp.glsl", fragmentOptions.get());
                osg::Shader* fvshader = osgDB::readShaderFile("gauss_convolution_1Dy_fp.glsl", fragmentOptions.get());

                // setup horizontal blur shaders
                gaussx->addShader(vshader);
                gaussx->addShader(fhshader);
                gaussx->setName("BlurHorizontalShader");

                gaussx->add("sigma", osg::Uniform::FLOAT);
                gaussx->add("radius", osg::Uniform::FLOAT);
                gaussx->add("texUnit0", osg::Uniform::SAMPLER_2D);

                gaussx->set("sigma", dofGaussSigma);
                gaussx->set("radius", dofGaussRadius);
                gaussx->set("texUnit0", 0);

                // setup vertical blur shaders
                gaussy->addShader(vshader);
                gaussy->addShader(fvshader);
                gaussy->setName("BlurVerticalShader");

                gaussy->add("sigma", osg::Uniform::FLOAT);
                gaussy->add("radius", osg::Uniform::FLOAT);
                gaussy->add("texUnit0", osg::Uniform::SAMPLER_2D);

                gaussy->set("sigma", dofGaussSigma);
                gaussy->set("radius", dofGaussRadius);
                gaussy->set("texUnit0", 0);
            }

            // now we perform a gauss blur on the downsampled data
            osgPPU::UnitInOut* blurxlight = new osgPPU::UnitInOut();
            osgPPU::UnitInOut* blurylight = new osgPPU::UnitInOut();
            {
                // set name and indicies
                blurxlight->setName("BlurHorizontalLight");
                blurylight->setName("BlurVerticalLight");

                //blurxlight->setShader(gaussx);
                //blurylight->setShader(gaussy);
                blurxlight->getOrCreateStateSet()->setAttributeAndModes(gaussx);
                blurylight->getOrCreateStateSet()->setAttributeAndModes(gaussy);
            }
            resampleLight->addChild(blurxlight);
            blurxlight->addChild(blurylight);


            // and also a stronger blurred/resampled texture is required
            osgPPU::UnitInResampleOut* resampleStrong = new osgPPU::UnitInResampleOut();
            {
                resampleStrong->setName("ResampleStrong");
                resampleStrong->setFactorX(0.25);
                resampleStrong->setFactorY(0.25);
            }
            bypass->addChild(resampleStrong);

            // now we perform a gauss blur on the downsampled data
            osgPPU::UnitInOut* blurxstrong = new osgPPU::UnitInOut();
            osgPPU::UnitInOut* blurystrong = new osgPPU::UnitInOut();
            {
                // set name and indicies
                blurxstrong->setName("BlurHorizontalStrong");
                blurystrong->setName("BlurVerticalStrong");

                //blurxstrong->setShader(gaussx);
                //blurystrong->setShader(gaussy);
                blurxstrong->getOrCreateStateSet()->setAttributeAndModes(gaussx);
                blurystrong->getOrCreateStateSet()->setAttributeAndModes(gaussy);
            }
            resampleStrong->addChild(blurxstrong);
            blurxstrong->addChild(blurystrong);


            // And finally we add a ppu which do use all the computed results:
            osgPPU::Unit* dof = new osgPPU::UnitInOut();
            {
                // setup inputs, name and index
                dof->setName("DoF-Result");

                // setup shader
                dofShaderAttr = new osgPPU::ShaderAttribute();
                dofShaderAttr->addShader(osgDB::readShaderFile("depth_of_field_fp.glsl", fragmentOptions.get()));
                dofShaderAttr->setName("DoFResultShader");

                dofShaderAttr->add("focalLength", osg::Uniform::FLOAT);
                dofShaderAttr->add("focalRange", osg::Uniform::FLOAT);
                dofShaderAttr->add("zNear", osg::Uniform::FLOAT);
                dofShaderAttr->add("zFar", osg::Uniform::FLOAT);

                dofShaderAttr->set("focalLength", dofFocalLength);
                dofShaderAttr->set("focalRange", dofFocalRange);
                dofShaderAttr->set("zNear", zNear);
                dofShaderAttr->set("zFar", zFar);

                //dof->setShader(sh);
                dof->getOrCreateStateSet()->setAttributeAndModes(dofShaderAttr);
                dof->setInputTextureIndexForViewportReference(0); // we want to setup viewport based on this input

                // add inputs as uniform parameters
                dof->setInputToUniform(bypass, "texColorMap", true);
                dof->setInputToUniform(blurylight, "texBlurredColorMap", true);
                dof->setInputToUniform(blurystrong, "texStrongBlurredColorMap", true);
                dof->setInputToUniform(depthbypass, "texDepthMap", true);
            }

            // this is the last unit
            lastUnit = dof;
        }
};

