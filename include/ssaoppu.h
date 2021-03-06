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

/**********************************************/
// PPU setup for SSAO (ambient occlusion) rendering
/**********************************************/
class SSAORendering : virtual public osg::Referenced
{
    private:
        float dofGaussSigma;
        float dofGaussRadius;

        osgPPU::ShaderAttribute* ssaoShaderAttr;
        osgPPU::ShaderAttribute* gaussx;
        osgPPU::ShaderAttribute* gaussy;
        osgPPU::ShaderAttribute* compositeAttr;

        osgPPU::UnitInResampleOut* mResampleNormal;   
        osgPPU::UnitInResampleOut* mResamplePosition;
        osgPPU::UnitInResampleOut* mResampleDepth;
        osgPPU::UnitInResampleOut* mResampleSSAO; 

    public:
        /********************/
        SSAORendering()
            :ssaoShaderAttr(NULL),
            gaussx(NULL),
            gaussy(NULL),
            compositeAttr(NULL)
        {
            dofGaussSigma = 1.5f;
            dofGaussRadius = 3.0f;
        }

        /*******************/
        void setPower(float pPower)
        {
            ssaoShaderAttr->set("vPower", pPower);
        }

        /*******************/
        void setGaussSigma(float sigma)
        {
            gaussx->set("sigma", sigma);
            gaussy->set("sigma", sigma);
        }

        /*******************/
        void setGaussRadius(float radius)
        {
            gaussx->set("radius", radius);
            gaussy->set("radius", radius);
        }

        /*******************/
        void setProjectionMatrix(osg::Matrixf pProjMat)
        {
            ssaoShaderAttr->set("vProjectionMatrix", pProjMat);
            ssaoShaderAttr->set("vInvProjectionMatrix", osg::Matrixf::inverse(pProjMat));
        }
    
        /*******************/
        void setSsaoPower(float pPower)
        {
            ssaoShaderAttr->set("vSsaoPower", pPower);
        }
        
        /*******************/
        void setSsaoFocus(float pFocus)
        {
            ssaoShaderAttr->set("vSsaoFocus", pFocus);
        }

        /*******************/
        void setSsaoSamples(int pSamples)
        {
            ssaoShaderAttr->set("vSsaoSamples", pSamples);
        }

        /*******************/
        void setResampleFactor(float pValue)
        {
            if(pValue < 1.f)
                return;

            float lInv = 1.f/pValue;

            mResampleNormal->setFactorX(lInv);
            mResampleNormal->setFactorY(lInv);
            mResampleNormal->dirty();
        
            mResamplePosition->setFactorX(lInv);
            mResamplePosition->setFactorY(lInv);
            mResamplePosition->dirty();

            mResampleDepth->setFactorX(lInv);
            mResampleDepth->setFactorY(lInv);
            mResampleDepth->dirty();
        
            mResampleSSAO->setFactorX(pValue);
            mResampleSSAO->setFactorY(pValue);
            mResampleSSAO->dirty();
        }

        /*******************/
        void createSSAOPipeline(osgPPU::Processor* pParent, osgPPU::Unit*& pLastUnit, osg::Matrixf pProjMat)
        {
            osg::ref_ptr<osgDB::ReaderWriter::Options> fragmentOptions = new osgDB::ReaderWriter::Options("fragment");            
            osg::ref_ptr<osgDB::ReaderWriter::Options> vertexOptions = new osgDB::ReaderWriter::Options("vertex");

            double fovy, aspect, lNear, lFar;
            pProjMat.getPerspective(fovy, aspect, lNear, lFar);

            // Now we are ready for the PPU
            // The first unit gets the first color buffer
            osgPPU::UnitCameraAttachmentBypass* lColor1 = new osgPPU::UnitCameraAttachmentBypass();
            lColor1->setBufferComponent(osg::Camera::COLOR_BUFFER0);
            lColor1->setName("Color");
            pParent->addChild(lColor1);

            // The second unit gets the second color buffer
            osgPPU::UnitCameraAttachmentBypass* lNormal = new osgPPU::UnitCameraAttachmentBypass();
            lNormal->setBufferComponent(osg::Camera::COLOR_BUFFER1);
            lNormal->setName("Normal");
            pParent->addChild(lNormal);
            
            // We dowmsample the normals
            mResampleNormal = new osgPPU::UnitInResampleOut();
            {
                mResampleNormal->setName("ResampleNormal");
                mResampleNormal->setFactorX(0.5f);
                mResampleNormal->setFactorY(0.5f);
            }
            lNormal->addChild(mResampleNormal);

            // The third unit gets the third color buffer
            osgPPU::UnitCameraAttachmentBypass* lPosition = new osgPPU::UnitCameraAttachmentBypass();
            lPosition->setBufferComponent(osg::Camera::COLOR_BUFFER2);
            lPosition->setName("Position");
            pParent->addChild(lPosition);

            // As well as the position
            mResamplePosition = new osgPPU::UnitInResampleOut();
            {
                mResamplePosition->setName("ResamplePosition");
                mResamplePosition->setFactorX(0.5f);
                mResamplePosition->setFactorY(0.5f);
            }
            lPosition->addChild(mResamplePosition);

            // The fourth unit gets the noise texture
            // It is read from an image file
            osg::Texture* lNoiseTex = new osg::Texture2D;
            lNoiseTex->setWrap(osg::Texture::WRAP_S, osg::Texture::MIRROR);
            lNoiseTex->setWrap(osg::Texture::WRAP_T, osg::Texture::MIRROR);
            lNoiseTex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST);
            lNoiseTex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::NEAREST);

            osg::ref_ptr<osg::Image> lNoiseImage = osgDB::readImageFile("ssao_noise.png");
            if(lNoiseImage.valid())
            {                
                lNoiseTex->setImage(0, lNoiseImage.get());
            }
            else
            {
                std::cout << "File ssao_noise.png not found. Unexpected results are to be expected concerning SSAO." << std::endl;
            }

            osgPPU::UnitTexture* lNoise = new osgPPU::UnitTexture();
            lNoise->setName("Noise");
            lNoise->setTexture(lNoiseTex);
            pParent->addChild(lNoise);

            // The 5th unit gets the depth buffer
            osgPPU::UnitDepthbufferBypass* lDepth = new osgPPU::UnitDepthbufferBypass();
            lDepth->setName("Depth");
            pParent->addChild(lDepth);

            // As well as the position
            mResampleDepth = new osgPPU::UnitInResampleOut();
            {
                mResampleDepth->setName("ResampleDepth");
                mResampleDepth->setFactorX(0.5f);
                mResampleDepth->setFactorY(0.5f);
            }
            lDepth->addChild(mResampleDepth);

            // And the unit to apply our shader
            osgPPU::UnitInOut* ssao = new osgPPU::UnitInOut();
            ssao->setName("ssao");
            
            ssaoShaderAttr = new osgPPU::ShaderAttribute();
            // Setup the ssao shader
            {
                ssaoShaderAttr->addShader(osgDB::readShaderFile("screenSpaceAmbientOcc_vp.glsl", vertexOptions.get()));
                ssaoShaderAttr->addShader(osgDB::readShaderFile("screenSpaceAmbientOcc_fp.glsl", fragmentOptions.get())); 
                ssaoShaderAttr->setName("ssaoShader");

                ssaoShaderAttr->add("vNear", osg::Uniform::FLOAT);
                ssaoShaderAttr->add("vFar", osg::Uniform::FLOAT);
                ssaoShaderAttr->add("vProjectionMatrix", osg::Uniform::FLOAT_MAT4);
                ssaoShaderAttr->add("vInvProjectionMatrix", osg::Uniform::FLOAT_MAT4);
                ssaoShaderAttr->add("vSsaoPower", osg::Uniform::FLOAT);
                ssaoShaderAttr->add("vSsaoFocus", osg::Uniform::FLOAT);
                ssaoShaderAttr->add("vSsaoSamples", osg::Uniform::INT);

                double lFovy, lRatio, lNear, lFar;
                pProjMat.getPerspective(lFovy, lRatio, lNear, lFar);

                ssaoShaderAttr->set("vNear", (float)lNear);
                ssaoShaderAttr->set("vFar", (float)lFar);
                ssaoShaderAttr->set("vProjectionMatrix", pProjMat);
                ssaoShaderAttr->set("vInvProjectionMatrix", osg::Matrixf::inverse(pProjMat));
                ssaoShaderAttr->set("vSsaoPower", 3.f);
                ssaoShaderAttr->set("vSsaoFocus", 6.f);
                ssaoShaderAttr->set("vSsaoSamples", 32);

                ssao->getOrCreateStateSet()->setAttributeAndModes(ssaoShaderAttr);
                ssao->setInputTextureIndexForViewportReference(0);

                //ssao->setInputToUniform(lColor1, "vColor1", true); // We don't need this anymore
                ssao->setInputToUniform(mResampleNormal, "vNormal", true);
                ssao->setInputToUniform(mResamplePosition, "vPosition", true);
                ssao->setInputToUniform(lNoise, "vNoiseMap", true);
                ssao->setInputToUniform(mResampleDepth, "vDepth", true);
            }

            // Now we will filter the ssao as it's quite noisy
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
            ssao->addChild(blurxlight);
            blurxlight->addChild(blurylight);

            // Resample to match the color FBO
            mResampleSSAO = new osgPPU::UnitInResampleOut();
            {
                mResampleSSAO->setName("ResampleSSAO");
                mResampleSSAO->setFactorX(2.0f);
                mResampleSSAO->setFactorY(2.0f);
            }
            blurylight->addChild(mResampleSSAO);


            // Last step: multiplication of the SSAO and the previous rendering
            osgPPU::UnitInOut* composite = new osgPPU::UnitInOut();
            compositeAttr = new osgPPU::ShaderAttribute();
            {
                compositeAttr->addShader(osgDB::readShaderFile("screenSpaceAO_composite_vp.glsl", vertexOptions.get()));
                compositeAttr->addShader(osgDB::readShaderFile("screenSpaceAO_composite_fp.glsl", fragmentOptions.get()));
                compositeAttr->setName("compositeSSAO");

                compositeAttr->add("vSSAO", osg::Uniform::SAMPLER_2D);
                compositeAttr->add("vColor", osg::Uniform::SAMPLER_2D);

                //compositeAttr->set("vSSAO", 0);
                
                composite->getOrCreateStateSet()->setAttributeAndModes(compositeAttr);

                composite->setInputToUniform(mResampleSSAO, "vSSAO", true);
                composite->setInputToUniform(lColor1, "vColor", true);
            }
            mResampleSSAO->addChild(composite);

            pLastUnit = composite;
        }
};
