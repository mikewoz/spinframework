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
        void createShaderPipeline(osgPPU::Processor* parent, osgPPU::Unit*& lastUnit)
        {
            osg::ref_ptr<osgDB::ReaderWriter::Options> fragmentOptions = new osgDB::ReaderWriter::Options("fragment");
            osg::ref_ptr<osgDB::ReaderWriter::Options> vertexOptions = new osgDB::ReaderWriter::Options("vertex");

            // If last unit is nullthe first unit will bypass the color output of the camera
            osgPPU::Unit* bypass;
            if (lastUnit == NULL)
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

            mShader->setInputToUniform(bypass, "vColor", true);

            bypass->addChild(mShader);
            lastUnit = mShader;
        }
};
