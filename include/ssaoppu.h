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
        osgPPU::ShaderAttribute* ssaoShaderAttr;    

    public:
        /********************/
        SSAORendering()
        {
        }

        /*******************/
        void setPower(float pPower)
        {
            ssaoShaderAttr->set("vPower", pPower);
        }

        /*******************/
        void createSSAOPipeline(osgPPU::Processor* pParent, osgPPU::Unit*& pLastUnit,
                                osg::Texture* pColor1, osg::Texture* pColor2, osg::Texture* pColor3,
                                osg::Matrixf pProjMat)
        {
            double fovy, aspect, lNear, lFar;
            pProjMat.getPerspective(fovy, aspect, lNear, lFar);
            std::cout << "zNear: " << lNear << " - zFar: " << lFar << std::endl;
                        
            osg::ref_ptr<osgDB::ReaderWriter::Options> fragmentOptions = new osgDB::ReaderWriter::Options("fragment");
            osg::ref_ptr<osgDB::ReaderWriter::Options> vertexOptions = new osgDB::ReaderWriter::Options("vertex");

            // We need to get the ptr to the texture attached to the camera
            //osg::Camera::BufferAttachmentMap lBufferMap = pCamera->getBufferAttachmentMap();
            //osg::Texture* lColorTexture1 = lBufferMap[osg::Camera::COLOR_BUFFER0]._texture;
            //osg::Texture* lColorTexture2 = lBufferMap[osg::Camera::COLOR_BUFFER1]._texture;

            // Now we are ready for the PPU
            // The first unit gets the first color buffer
            osgPPU::UnitTexture* lColor1 = new osgPPU::UnitTexture();
            lColor1->setName("Color1");
            lColor1->setTexture(pColor1);
            pParent->addChild(lColor1);

            // The second unit gets the second color buffer
            osgPPU::UnitTexture* lColor2 = new osgPPU::UnitTexture();
            lColor2->setName("Color2");
            lColor2->setTexture(pColor2);
            pParent->addChild(lColor2);

            // The third unit gets the third color buffer
            osgPPU::UnitTexture* lColor3 = new osgPPU::UnitTexture();
            lColor3->setName("Color3");
            lColor3->setTexture(pColor3);
            pParent->addChild(lColor3);

            // The fourth unit gets the noise texture
            // It is read from an image file
            osg::ref_ptr<osg::Image> lNoiseImage = osgDB::readImageFile("ssao_noise.png");
            osg::Texture* lNoiseTex = new osg::Texture2D;
            lNoiseTex->setWrap(osg::Texture::WRAP_S, osg::Texture::MIRROR);
            lNoiseTex->setWrap(osg::Texture::WRAP_T, osg::Texture::MIRROR);
            lNoiseTex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST);
            lNoiseTex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::NEAREST);
            lNoiseTex->setImage(0, lNoiseImage.get());

            osgPPU::UnitTexture* lNoise = new osgPPU::UnitTexture();
            lNoise->setName("Noise");
            lNoise->setTexture(lNoiseTex);
            pParent->addChild(lNoise);

            // The 5th unit gets the depth buffer
            osgPPU::UnitDepthbufferBypass* lDepth = new osgPPU::UnitDepthbufferBypass();
            lDepth->setName("Depth");
            pParent->addChild(lDepth);

            // And the unit to apply our shader
            osgPPU::Unit* ssao = new osgPPU::UnitInOut();
            ssao->setName("ssao");
            
            // Setup the shader
            ssaoShaderAttr = new osgPPU::ShaderAttribute();
            ssaoShaderAttr->addShader(osgDB::readShaderFile("screenSpaceAmbientOcc_vp.glsl", vertexOptions.get()));
            ssaoShaderAttr->addShader(osgDB::readShaderFile("screenSpaceAmbientOcc_fp.glsl", fragmentOptions.get())); 
            ssaoShaderAttr->setName("ssaoShader");

            ssaoShaderAttr->add("vPower", osg::Uniform::FLOAT);
            ssaoShaderAttr->add("vPixelSize", osg::Uniform::FLOAT);
            ssaoShaderAttr->add("vNear", osg::Uniform::FLOAT);
            ssaoShaderAttr->add("vFar", osg::Uniform::FLOAT);
            ssaoShaderAttr->add("vProjectionMatrix", osg::Uniform::FLOAT_MAT4);

            ssaoShaderAttr->set("vPixelSize", 1.f);
            ssaoShaderAttr->set("vNear", (float)lNear);
            ssaoShaderAttr->set("vFar", (float)lFar);
            ssaoShaderAttr->set("vProjectionMatrix", pProjMat);

            ssao->getOrCreateStateSet()->setAttributeAndModes(ssaoShaderAttr);
            ssao->setInputTextureIndexForViewportReference(0);

            ssao->setInputToUniform(lColor1, "vColor1", true);
            ssao->setInputToUniform(lColor2, "vColor2", true);
            ssao->setInputToUniform(lColor3, "vColor3", true);
            ssao->setInputToUniform(lNoise, "vNoiseMap", true);
            ssao->setInputToUniform(lDepth, "vDepth", true);

            pLastUnit = ssao;
        }
};
