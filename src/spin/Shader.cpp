// -----------------------------------------------------------------------------
// |    ___  ___  _  _ _     ___                                        _      |
// |   / __>| . \| || \ |   | __>_ _  ___ ._ _ _  ___  _ _ _  ___  _ _ | |__   |
// |   \__ \|  _/| ||   |   | _>| '_><_> || ' ' |/ ._>| | | |/ . \| '_>| / /   |
// |   <___/|_|  |_||_\_|   |_| |_|  <___||_|_|_|\___.|__/_/ \___/|_|  |_\_\   |
// |                                                                           |
// |---------------------------------------------------------------------------|
//
// http://spinframework.sourceforge.net
// Copyright (C) 2009 Mike Wozniewski, Zack Settel
//
// Developed/Maintained by:
//    Mike Wozniewski (http://www.mikewoz.com)
//    Zack Settel (http://www.sheefa.net/zack)
//
// Principle Partners:
//    Shared Reality Lab, McGill University (http://www.cim.mcgill.ca/sre)
//    La Societe des Arts Technologiques (http://www.sat.qc.ca)
//
// Funding by:
//    NSERC/Canada Council for the Arts - New Media Initiative
//    Heritage Canada
//    Ministere du Developpement economique, de l'Innovation et de l'Exportation
//
// -----------------------------------------------------------------------------
//  This file is part of the SPIN Framework.
//
//  SPIN Framework is free software: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  SPIN Framework is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  along with SPIN Framework. If not, see <http://www.gnu.org/licenses/>.
// -----------------------------------------------------------------------------

#include <osg/StateSet>
#include <osg/StateAttribute>
#include <osg/TextureRectangle>
#include <osg/Texture2D>
#include <osg/Texture3D>
#include <osgDB/ReadFile>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>

#include <osg/Program>
#include <osg/Shader>
#include <osg/Uniform>

#include <iostream>

#include "SceneManager.h"
#include "Shader.h"
#include "ShaderUtil.h"
#include "spinApp.h"
#include "spinBaseContext.h"

extern pthread_mutex_t sceneMutex;

namespace spin
{


static bool loadShaderSource(osg::Shader* obj, const std::string& fileName )
{
   std::string fqFileName = osgDB::findDataFile(fileName);
   if( fqFileName.length() == 0 )
   {
      std::cout << "Warning: Shader file \"" << fileName << "\" not found." << std::endl;
      return false;
   }
   bool success = obj->loadShaderSourceFromFile( fqFileName.c_str());
   if ( !success  )
   {
      std::cout << "Warning: Shader could not be loaded: " << fileName << std::endl;
      return false;
   }
   else
   {
      return true;
   }
}

static osg::Image*
make3DNoiseImage(int texSize)
{
    osg::Image* image = new osg::Image;
    image->setImage(texSize, texSize, texSize,
            4, GL_RGBA, GL_UNSIGNED_BYTE,
            new unsigned char[4 * texSize * texSize * texSize],
            osg::Image::USE_NEW_DELETE);

    const int startFrequency = 4;
    const int numOctaves = 4;

    int f, i, j, k, inc;
    double ni[3];
    double inci, incj, inck;
    int frequency = startFrequency;
    GLubyte *ptr;
    double amp = 0.5;

    osg::notify(osg::INFO) << "creating 3D noise texture... ";

    for (f = 0, inc = 0; f < numOctaves; ++f, frequency *= 2, ++inc, amp *= 0.5)
    {
        SetNoiseFrequency(frequency);
        ptr = image->data();
        ni[0] = ni[1] = ni[2] = 0;

        inci = 1.0 / (texSize / frequency);
        for (i = 0; i < texSize; ++i, ni[0] += inci)
        {
            incj = 1.0 / (texSize / frequency);
            for (j = 0; j < texSize; ++j, ni[1] += incj)
            {
                inck = 1.0 / (texSize / frequency);
                for (k = 0; k < texSize; ++k, ni[2] += inck, ptr += 4)
                {
                    *(ptr+inc) = (GLubyte) (((noise3(ni) + 1.0) * amp) * 128.0);
                }
            }
        }
    }

    osg::notify(osg::INFO) << "DONE" << std::endl;
    return image;
}

static osg::Texture3D*
make3DNoiseTexture(int texSize )
{
    osg::Texture3D* noiseTexture = new osg::Texture3D;
    noiseTexture->setFilter(osg::Texture3D::MIN_FILTER, osg::Texture3D::LINEAR);
    noiseTexture->setFilter(osg::Texture3D::MAG_FILTER, osg::Texture3D::LINEAR);
    noiseTexture->setWrap(osg::Texture3D::WRAP_S, osg::Texture3D::REPEAT);
    noiseTexture->setWrap(osg::Texture3D::WRAP_T, osg::Texture3D::REPEAT);
    noiseTexture->setWrap(osg::Texture3D::WRAP_R, osg::Texture3D::REPEAT);
    noiseTexture->setImage( make3DNoiseImage(texSize) );
    return noiseTexture;
}

#define TEXUNIT_NOISE 2

// *****************************************************************************
// constructor:
Shader::Shader (SceneManager *s, const char *initID) : ReferencedStateSet(s, initID)
{
	classType = "Shader";

	_path = "NULL";
	_renderBin = 11;
    _updateRate = 1.0;
	_lightingEnabled = true;
    _programObject = new osg::Program;

    // NEEDED FOR TESTS:
    /*
    _vertexObject = new osg::Shader( osg::Shader::VERTEX );
    _fragmentObject = new osg::Shader( osg::Shader::FRAGMENT );
    _programObject->addShader( _fragmentObject );
    _programObject->addShader( _vertexObject );
    */

    // TEST 1
    /*
    loadShaderSource( _vertexObject, "/Users/mikewoz/src/OpenSceneGraph-Data-3.0.0/shaders/brick.vert" );
    loadShaderSource( _fragmentObject, "/Users/mikewoz/src/OpenSceneGraph-Data-3.0.0/shaders/brick.frag" );
    */

    // TEST 2
    /*
    osg::Texture3D* noiseTexture = make3DNoiseTexture( 32 ); // 128 );
    //osg::Texture1D* sineTexture = make1DSineTexture( 32 ); // 1024 );
    
    this->setTextureAttribute(TEXUNIT_NOISE, noiseTexture);
    loadShaderSource( _vertexObject, "/Users/mikewoz/src/OpenSceneGraph-Data-3.0.0/shaders/eroded.vert" );
    loadShaderSource( _fragmentObject, "/Users/mikewoz/src/OpenSceneGraph-Data-3.0.0/shaders/eroded.frag" );
    this->addUniform( new osg::Uniform("LightPosition", osg::Vec3(0.0f, 0.0f, 4.0f)) );
    this->addUniform( new osg::Uniform("Scale", 1.0f) );
    this->addUniform( new osg::Uniform("Sine", 1.0f) );
    this->addUniform( new osg::Uniform("sampler3d", TEXUNIT_NOISE) );
    this->addUniform( new osg::Uniform("Offset", osg::Vec3(0.0f, 0.0f, 0.0f)) );
    */
    
    // TEST 3:
    /*
    _programObject->setName( "blocky" );
    loadShaderSource( _vertexObject,   "/Users/mikewoz/src/spinframework/src/Resources/shaders/blocky.vert" );
    loadShaderSource( _fragmentObject, "/Users/mikewoz/src/spinframework/src/Resources/shaders/blocky.frag" );
    this->addUniform( new osg::Uniform( "Sine", 0.0f ) );
    */
    
    this->setAttributeAndModes(_programObject, osg::StateAttribute::ON);
}

// destructor
Shader::~Shader()
{
}

// *****************************************************************************


void Shader::updateCallback()
{
    //if (!sceneManager->isGraphical()) return;


    

    // do something every _updateRate seconds:
    osg::Timer_t tick = osg::Timer::instance()->tick();
    float dt = osg::Timer::instance()->delta_s(lastTick,tick);
    if (dt > _updateRate)
    {
        

        // this is the last time we checked
        lastTick = osg::Timer::instance()->tick();
    }


}

// *****************************************************************************
void Shader::debug()
{
	ReferencedStateSet::debug();
	
	std::cout << "   ---------" << std::endl;

	// additional info
	if (_vertexObject.valid())
	{
		std::cout << "   vert shader: " << _vertexObject->getShaderSource() << std::endl;
	} else {
        std::cout << "   vert shader: null" << std::endl;
    }
	if (_fragmentObject.valid())
	{
		std::cout << "   frag shader: " << _fragmentObject->getShaderSource() << std::endl;
	} else {
        std::cout << "   frag shader: null" << std::endl;
    }
    
    //typedef std::map< std::string, RefUniformPair > UniformList
    
    const osg::StateSet::UniformList uniforms = this->getUniformList();
 	osg::StateSet::UniformList::const_iterator uitr;
    for (uitr=uniforms.begin(); uitr!=uniforms.end(); ++uitr)
    {
        const std::string& name = uitr->first;
        osg::Uniform *u = uitr->second.first;
        std::cout << "   uniform " << name;
        
        osg::Vec3 v3;
        switch (u->getType())
        {
            case osg::Uniform::FLOAT:
                float f;
                u->get(f);
                std::cout << " (type: FLOAT) = " << f;
                break;
            case osg::Uniform::FLOAT_VEC3:
                u->get(v3);
                std::cout << " (type: VEC3) = " << stringify(v3);
                break;
            case osg::Uniform::INT:
                int i;
                u->get(i);
                std::cout << " (type: INT) = " << i;
                break;
            case osg::Uniform::BOOL:
                bool b;
                u->get(b);
                std::cout << " (type: BOOL) = " << b;
                break;

            default:
                std::cout << " (type: OTHER)";
                break;
        }
        std::cout << std::endl;
    }
}


// *****************************************************************************
void Shader::setUniform_Float (const char* name, float f)
{
    osg::Uniform *u = this->getUniform(name);
    if (u) u->set(f);
    
    BROADCAST(this, "ssf", "setUniform_Float", name, f);
}

void Shader::setUniform_Vec3 (const char* name, float x, float y, float z)
{
    osg::Vec3 v = osg::Vec3(x,y,z);
    osg::Uniform *u = this->getUniform(name);
    if (u) u->set(v);
    
    BROADCAST(this, "ssfff", "setUniform_Vec3", name, x, y, z);
}


// *****************************************************************************
void Shader::setPath (const char* newPath)
{
	// only do this if the id has changed:
	if (_path == std::string(newPath)) return;

	_path = std::string(newPath);
    

	
	if (sceneManager->isGraphical())
	{
        // remove previous shader:
        //pthread_mutex_lock(&sceneMutex);
        if (_vertexObject)
        {
            _programObject->removeShader(_vertexObject);
            _vertexObject = 0;
        }
        if (_fragmentObject)
        {
            _programObject->removeShader(_fragmentObject);
            _fragmentObject = 0;
        }
        //pthread_mutex_unlock(&sceneMutex);
        
        if (_path == "NULL")
        {
            std::cout << "Disabling shader '"<< this->id->s_name << "'" << std::endl;
        }
        else
        {
            // create new shaders:
            osg::Shader *vertShader = new osg::Shader( osg::Shader::VERTEX );
            osg::Shader *fragShader = new osg::Shader( osg::Shader::FRAGMENT );

            std::string fullPath = getAbsolutePath(_path);
            std::cout << "Loading shader: " << fullPath << std::endl;

            std::string folderPath = osgDB::getFilePath(fullPath);

            bool vertSuccess = false;
            bool fragSuccess = false;
            std::string ext = osgDB::getLowerCaseFileExtension(fullPath);
            if (ext=="frag")
            {
                fragSuccess = loadShaderSource( fragShader, fullPath );
                vertSuccess = loadShaderSource( vertShader, folderPath+".vert" );
            }
            else if (ext=="vert")
            {
                vertSuccess = loadShaderSource( vertShader, fullPath );
                fragSuccess = loadShaderSource( fragShader, folderPath+".frag" );
            }
            else
            {
                // let's check if the user just passed the base file name (without an extention)
                vertSuccess = loadShaderSource( vertShader, fullPath+".vert" );
                fragSuccess = loadShaderSource( fragShader, fullPath+".frag" );
            }
            
            // set name:
            _programObject->setName(osgDB::getStrippedName(fullPath));

            // add shaders to program:
            if (vertSuccess)
            {
                _vertexObject = vertShader;
                _programObject->addShader( _vertexObject );
            }
            if (fragSuccess)
            {
                _fragmentObject = fragShader;
                _programObject->addShader( _fragmentObject );
            }
        }

        // enable shader:
        //if (vertSuccess && fragSuccess)
        //    this->setAttributeAndModes(_programObject, osg::StateAttribute::ON);
        _programObject->dirtyProgram();


        // set lighting:
        if (_lightingEnabled) this->setMode( GL_LIGHTING, osg::StateAttribute::ON );
        else this->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

        // set renderbin:
        this->setRenderBinDetails( _renderBin, "RenderBin");

	}

	BROADCAST(this, "ss", "setPath", getPath());
}

void Shader::setLighting (int i)
{
	_lightingEnabled = (bool)i;

	if (_lightingEnabled) this->setMode( GL_LIGHTING, osg::StateAttribute::ON );
	else this->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

	BROADCAST(this, "si", "setLighting", getLighting());
}

void Shader::setRenderBin (int i)
{
	_renderBin = i;
	this->setRenderBinDetails( (int)_renderBin, "RenderBin");

	BROADCAST(this, "si", "setRenderBin", getRenderBin());
}

// *****************************************************************************
std::vector<lo_message> Shader::getState () const
{
	// inherit state from base class
	std::vector<lo_message> ret = ReferencedStateSet::getState();
		
	lo_message msg;
	
	msg = lo_message_new();
	lo_message_add(msg, "ss", "setPath", getPath());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "si", "setLighting", getLighting());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "si", "setRenderBin", getRenderBin());
	ret.push_back(msg);
	
	return ret;
}

} // end of namespace spin

