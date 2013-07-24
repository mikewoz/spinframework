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
#include <osgPPU/UnitCamera.h>
#include <osgDB/ReaderWriter>
#include <osgDB/ReadFile>
#include <osgPPU/ShaderAttribute.h>

/********************************************/
// PPU which adds an outline to all objects,
// through an edge detection on the depth map
/********************************************/
class OutlineRendering : virtual public osg::Referenced
{
    private:
        osgPPU::ShaderAttribute* outlineAttr;
        osgPPU::ShaderAttribute* compositeAttr;
        osgPPU::ShaderAttribute* dilatexAttr;
        osgPPU::ShaderAttribute* dilateyAttr;
        osgPPU::ShaderAttribute* gaussx;
        osgPPU::ShaderAttribute* gaussy;

        osgPPU::UnitInResampleOut* mResample;
        osgPPU::UnitInResampleOut* mUnResample;

        float dilateRadius;
        float glowRadius, glowSigma;

    public:
        /*********/
        OutlineRendering():
            outlineAttr(NULL),
            compositeAttr(NULL),
            dilatexAttr(NULL),
            dilateyAttr(NULL),
            gaussx(NULL),
            gaussy(NULL)
        {
            dilateRadius = 3.0f;
            glowRadius = 10.f;
            glowSigma = 5.f;
        }

        /*******************/
        void setOutlineMode(int mode)
        {
            if(mode < 0 || mode > 2)
                return;

            outlineAttr->set("uMode", mode);
        }

        /*******************/
        void setOutlineStrength(float radius)
        {
            dilatexAttr->set("radius", radius);
            dilateyAttr->set("radius", radius);
        }

        /*******************/
        void setGlowSigma(float sigma)
        {
            gaussx->set("sigma", sigma);
            gaussy->set("sigma", sigma);
        }

        /*******************/
        void setGlowRadius(float radius)
        {
            gaussx->set("radius", radius);
            gaussy->set("radius", radius);
        }

        /****************/
        void setGlowPower(float power)
        {
            compositeAttr->set("uGlowPower", power);
        }

        /****************/
        void setOutlineColor(float r, float g, float b, float a)
        {
            compositeAttr->set("uOutlineColor", r, g, b, a);
        }

        /**************/
        void setResampleFactor(float pValue)
        {
            float lValue;
            if(pValue < 1.f)
                lValue = 1.f;
            else
                lValue = pValue;

            mResample->setFactorX(1.f/lValue);
            mResample->setFactorY(1.f/lValue);
            mResample->dirty();

            mUnResample->setFactorX(lValue);
            mUnResample->setFactorY(lValue);
            mUnResample->dirty();
        }
    
        /*********/
        void createOutlinePipeline(osgPPU::Processor* pParent, osgPPU::Unit*& pLastUnit, float zNear, float zFar)
        {
            osg::ref_ptr<osgDB::ReaderWriter::Options> fragmentOptions = new osgDB::ReaderWriter::Options("fragment");            
            osg::ref_ptr<osgDB::ReaderWriter::Options> vertexOptions = new osgDB::ReaderWriter::Options("vertex");

            // We get the color buffer
            osgPPU::Unit* lColor;
            if(pLastUnit == NULL)
            {
                lColor = new osgPPU::UnitCameraAttachmentBypass();
                ((osgPPU::UnitCameraAttachmentBypass*)lColor)->setBufferComponent(osg::Camera::COLOR_BUFFER0);
                ((osgPPU::UnitCameraAttachmentBypass*)lColor)->setName("color");
                pParent->addChild(lColor);
            }
            else
            {
                lColor = pLastUnit;
            }

            osgPPU::Unit* lDepth = new osgPPU::UnitDepthbufferBypass();
            lDepth->setName("depth");
            pParent->addChild(lDepth);
            
            // First we detect the edges on the depth map
            osgPPU::Unit* lEdges = new osgPPU::UnitInOut();
            outlineAttr = new osgPPU::ShaderAttribute();
            {
                osg::Shader* lVShader = osgDB::readShaderFile("outline_vp.glsl", vertexOptions.get());
                osg::Shader* lFShader = osgDB::readShaderFile("outline_fp.glsl", fragmentOptions.get());

                outlineAttr->addShader(lVShader);
                outlineAttr->addShader(lFShader);
                outlineAttr->setName("OutlineShader");

                outlineAttr->add("uPass", osg::Uniform::INT);
                outlineAttr->add("uMode", osg::Uniform::INT);
                outlineAttr->add("uNear", osg::Uniform::FLOAT);
                outlineAttr->add("uFar", osg::Uniform::FLOAT);

                outlineAttr->set("uPass", 1);
                outlineAttr->set("uMode", 0);
                outlineAttr->set("uNear", zNear);
                outlineAttr->set("uFar", zFar);

                lEdges->setName("edges");
                lEdges->getOrCreateStateSet()->setAttributeAndModes(outlineAttr);
                lEdges->setInputToUniform(lDepth, "uDepthMap", true);
                lEdges->setInputToUniform(lColor, "uColorMap", true);
            }

            // A dilate filter is applied to control the strength of the outline
            dilatexAttr = new osgPPU::ShaderAttribute();
            dilateyAttr = new osgPPU::ShaderAttribute();
            {
                // read shaders from file
                osg::Shader* vshader = osgDB::readShaderFile("dilate_vp.glsl", vertexOptions.get());
                osg::Shader* fhshader = osgDB::readShaderFile("dilate_1Dx_fp.glsl", fragmentOptions.get());
                osg::Shader* fvshader = osgDB::readShaderFile("dilate_1Dy_fp.glsl", fragmentOptions.get());

                // setup horizontal blur shaders
                dilatexAttr->addShader(vshader);
                dilatexAttr->addShader(fhshader);
                dilatexAttr->setName("DilateHorizontalShader");

                dilatexAttr->add("radius", osg::Uniform::FLOAT);
                dilatexAttr->add("texUnit0", osg::Uniform::SAMPLER_2D);

                dilatexAttr->set("radius", dilateRadius);
                dilatexAttr->set("texUnit0", 0);

                // setup vertical blur shaders
                dilateyAttr->addShader(vshader);
                dilateyAttr->addShader(fvshader);
                dilateyAttr->setName("DilateVerticalShader");

                dilateyAttr->add("radius", osg::Uniform::FLOAT);
                dilateyAttr->add("texUnit0", osg::Uniform::SAMPLER_2D);

                dilateyAttr->set("radius", dilateRadius);
                dilateyAttr->set("texUnit0", 0);
            }

            osgPPU::UnitInOut* lDilatex = new osgPPU::UnitInOut();
            osgPPU::UnitInOut* lDilatey = new osgPPU::UnitInOut();
            {
                // set name and indicies
                lDilatex->setName("DilateHorizontal");
                lDilatey->setName("DilateVertical");

                lDilatex->getOrCreateStateSet()->setAttributeAndModes(dilatexAttr);
                lDilatey->getOrCreateStateSet()->setAttributeAndModes(dilateyAttr);
            }
            lEdges->addChild(lDilatex);
            lDilatex->addChild(lDilatey);

            // For those who need glow, here it is !
            // We calculate it in a lowered resolution to keep performance high
            // It doesn't matter as glow is blur...
            mResample = new osgPPU::UnitInResampleOut();
            mResample->setName("resample");
            mResample->setFactorX(0.5f);
            mResample->setFactorY(0.5f);
            lDilatey->addChild(mResample);

            gaussx = new osgPPU::ShaderAttribute();
            gaussy = new osgPPU::ShaderAttribute();
            {
                // read shaders from file
                osg::Shader* lVertexShader = osgDB::readShaderFile("gauss_convolution_vp.glsl", vertexOptions.get());
                osg::Shader* lFHShader = osgDB::readShaderFile("gauss_convolution_1Dx_fp.glsl", fragmentOptions.get());
                osg::Shader* lFVShader = osgDB::readShaderFile("gauss_convolution_1Dy_fp.glsl", fragmentOptions.get());

                // setup horizontal blur shaders
                gaussx->addShader(lVertexShader);
                gaussx->addShader(lFHShader);
                gaussx->setName("BlurHorizontalShader");

                gaussx->add("sigma", osg::Uniform::FLOAT);
                gaussx->add("radius", osg::Uniform::FLOAT);
                gaussx->add("texUnit0", osg::Uniform::SAMPLER_2D);

                gaussx->set("sigma", glowSigma);
                gaussx->set("radius", glowRadius);
                gaussx->set("texUnit0", 0);

                // setup vertical blur shaders
                gaussy->addShader(lVertexShader);
                gaussy->addShader(lFVShader);
                gaussy->setName("BlurVerticalShader");

                gaussy->add("sigma", osg::Uniform::FLOAT);
                gaussy->add("radius", osg::Uniform::FLOAT);
                gaussy->add("texUnit0", osg::Uniform::SAMPLER_2D);

                gaussy->set("sigma", glowSigma);
                gaussy->set("radius", glowRadius);
                gaussy->set("texUnit0", 0);
            }

            osgPPU::UnitInOut* lGlowX = new osgPPU::UnitInOut();
            osgPPU::UnitInOut* lGlowY = new osgPPU::UnitInOut();
            {
                // set name and indicies
                lGlowX->setName("GlowHorizontalLight");
                lGlowY->setName("GlowVerticalLight");

                lGlowX->getOrCreateStateSet()->setAttributeAndModes(gaussx);
                lGlowY->getOrCreateStateSet()->setAttributeAndModes(gaussy);
            }
            mResample->addChild(lGlowX);
            lGlowX->addChild(lGlowY);

            // Go back to the original resolution
            mUnResample = new osgPPU::UnitInResampleOut();
            mUnResample->setName("unResample");
            mUnResample->setFactorX(2.0f);
            mUnResample->setFactorY(2.0f);
            lGlowY->addChild(mUnResample);

            // New we can composite the color image with the outline
            osgPPU::UnitInOut* lComposite = new osgPPU::UnitInOut();
            compositeAttr = new osgPPU::ShaderAttribute();
            {
                osg::Shader* lVShader = osgDB::readShaderFile("outline_vp.glsl", vertexOptions.get());
                osg::Shader* lFShader = osgDB::readShaderFile("outline_fp.glsl", fragmentOptions.get());

                compositeAttr->addShader(lVShader);
                compositeAttr->addShader(lFShader);
                compositeAttr->setName("CompositeShader");

                compositeAttr->add("uPass", osg::Uniform::INT);
                compositeAttr->add("uOutlineColor", osg::Uniform::FLOAT_VEC4);
                compositeAttr->add("uGlowPower", osg::Uniform::FLOAT);

                compositeAttr->set("uPass", 2);
                compositeAttr->set("uOutlineColor", 1.f, 1.f, 1.f, 1.f);
                compositeAttr->set("uGlowPower", 1.f);

                lComposite->setName("composite");
                lComposite->getOrCreateStateSet()->setAttributeAndModes(compositeAttr);
                lComposite->setInputToUniform(lDilatey, "uOutlineMap", true);
                lComposite->setInputToUniform(mUnResample, "uGlowMap", true);
                lComposite->setInputToUniform(lColor, "uColorMap", true);
            }

            pLastUnit = lComposite;
        }
};
