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

/*********************************************/
// This simple PPU allows the user to specify
// his own post-processing shader
/*********************************************/
class ShaderRendering : virtual public osg::Referenced
{
    private:
        std::string mShaderBaseName;
        osgPPU::ShaderAttribute* mShaderAttr;
        
        osgPPU::Unit* mShader;

    public:

        /****************************/
        ShaderRendering()
            : mShader(NULL),
            mShaderAttr(NULL)
        {
            mShaderBaseName = "shader";
        }

        /****************************/
        void setShaderFile(std::string filename)
        {
            osg::ref_ptr<osgDB::ReaderWriter::Options> fragmentOptions = new osgDB::ReaderWriter::Options("fragment");
            osg::ref_ptr<osgDB::ReaderWriter::Options> vertexOptions = new osgDB::ReaderWriter::Options("vertex");
            
            std::string vertexShaderFile = filename + std::string(".vert");
            std::string fragmentShaderFile = filename + std::string(".frag");

            osg::Shader* vShader = osgDB::readShaderFile(vertexShaderFile.c_str(), vertexOptions.get());
            osg::Shader* fShader = osgDB::readShaderFile(fragmentShaderFile.c_str(), fragmentOptions.get());

            if (vShader == 0 || fShader == 0)
            {
                std::cout << "Unable to load shader " << filename << std::endl;
                return;
            }

            // Remove old shaders
            while (mShaderAttr->getNumShaders() > 0)
                mShaderAttr->removeShader(mShaderAttr->getShader(0));
       
            mShaderAttr->addShader(vShader);
            mShaderAttr->addShader(fShader);

            mShader->dirty();
        }

        /****************************/
        void setShaderColor(std::string uniformName, float r, float g, float b, float a)
        {
            mShaderAttr->add(uniformName.c_str(), osg::Uniform::FLOAT_VEC4);
            mShaderAttr->set(uniformName.c_str(), r, g, b, a); 
        }

        /****************************/
        void createShaderPipeline(osgPPU::Processor* parent, osgPPU::Unit*& lastUnit)
        {
            osg::ref_ptr<osgDB::ReaderWriter::Options> fragmentOptions = new osgDB::ReaderWriter::Options("fragment");
            osg::ref_ptr<osgDB::ReaderWriter::Options> vertexOptions = new osgDB::ReaderWriter::Options("vertex");

            // If last unit is nullthe first unit will bypass the color output of the camera
            osgPPU::Unit* colorBuffer[4];
            if (lastUnit == NULL)
            {
                colorBuffer[0] = new osgPPU::UnitCameraAttachmentBypass();
                ((osgPPU::UnitCameraAttachmentBypass*)colorBuffer[0])->setBufferComponent(osg::Camera::COLOR_BUFFER0);
                ((osgPPU::UnitCameraAttachmentBypass*)colorBuffer[0])->setName("colorBuffer_0");
                parent->addChild(colorBuffer[0]);
            }
            else
            {
                colorBuffer[0] = lastUnit;
            }

            // We get to other buffers from the camera
            colorBuffer[1] = new osgPPU::UnitCameraAttachmentBypass();
            ((osgPPU::UnitCameraAttachmentBypass*)colorBuffer[1])->setBufferComponent(osg::Camera::COLOR_BUFFER1);
            parent->addChild(colorBuffer[1]);

            colorBuffer[2] = new osgPPU::UnitCameraAttachmentBypass();
            ((osgPPU::UnitCameraAttachmentBypass*)colorBuffer[2])->setBufferComponent(osg::Camera::COLOR_BUFFER2);
            parent->addChild(colorBuffer[2]);

            colorBuffer[3] = new osgPPU::UnitCameraAttachmentBypass();
            ((osgPPU::UnitCameraAttachmentBypass*)colorBuffer[3])->setBufferComponent(osg::Camera::COLOR_BUFFER3);
            parent->addChild(colorBuffer[3]);


            // If we are unable to open the specified shader, we do nothing
            std::string vertexShaderFile = mShaderBaseName + std::string(".vert");
            std::string fragmentShaderFile = mShaderBaseName + std::string(".frag");

            osg::Shader* vShader = osgDB::readShaderFile(vertexShaderFile.c_str(), vertexOptions.get());
            osg::Shader* fShader = osgDB::readShaderFile(fragmentShaderFile.c_str(), fragmentOptions.get());

            mShader = new osgPPU::UnitInOut();
            mShaderAttr = new osgPPU::ShaderAttribute();

            if (vShader == 0 || fShader == 0)
            {
                std::cout << "Unable to load shader " << mShaderBaseName << std::endl;
            }
            else
            {
                mShaderAttr->addShader(vShader);
                mShaderAttr->addShader(fShader);
            }

            mShader->getOrCreateStateSet()->setAttributeAndModes(mShaderAttr);

            mShader->setInputToUniform(colorBuffer[0], "vColorBuffer_0", true);
            mShader->setInputToUniform(colorBuffer[1], "vColorBuffer_1", true);
            mShader->setInputToUniform(colorBuffer[2], "vColorBuffer_2", true);
            mShader->setInputToUniform(colorBuffer[3], "vColorBuffer_3", true);

            colorBuffer[0]->addChild(mShader);
            lastUnit = mShader;
        }
};
