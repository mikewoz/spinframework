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

// *****************************************************************************
// constructor:
Shader::Shader (SceneManager *s, const char *initID) : ReferencedStateSet(s, initID)
{
	classType = "Shader";

	shaderPath_ = "NULL";
    updateRate_ = 1.0;
    programObject_ = new osg::Program;

    // SIMPLE TEST (colors each vertex according to position)
    /*
    vertexObject_ = new osg::Shader( osg::Shader::VERTEX, microshaderVertSource );
    fragmentObject_ = new osg::Shader( osg::Shader::FRAGMENT, microshaderFragSource );
    programObject_->addShader( fragmentObject_ );
    programObject_->addShader( vertexObject_ );
    */

    // test textures:
    if (0)
	{
        this->setTextureAttributeAndModes(0, new osg::TextureRectangle(osgDB::readImageFile( "/Users/mikewoz/src/pdsheefa/examples/Resources/lenna.jpg" )), osg::StateAttribute::ON);
        this->setTextureAttributeAndModes(1, new osg::Texture2D(osgDB::readImageFile( "/Users/mikewoz/src/pdsheefa/examples/Resources/sampleTexture1.jpg" )), osg::StateAttribute::ON);
	
    }
    if (0)
    {
        this->setTextureAttributeAndModes(0, new osg::TextureRectangle(osgDB::readImageFile( "/Users/mikewoz/src/pdsheefa/examples/Resources/lenna.jpg" )), osg::StateAttribute::ON);
        this->setTextureAttributeAndModes(1, new osg::Texture2D(osgDB::readImageFile( "/Users/mikewoz/src/pdsheefa/examples/Resources/sampleTexture1.jpg" )), osg::StateAttribute::ON);
    }

    //this->setAttributeAndModes(programObject_);
}

// destructor
Shader::~Shader()
{
}

// *****************************************************************************


void Shader::updateCallback()
{
    //if (!sceneManager->isGraphical()) return;

    // do something every updateRate_ seconds:
    osg::Timer_t tick = osg::Timer::instance()->tick();
    float dt = osg::Timer::instance()->delta_s(lastTick_,tick);
    if (dt > updateRate_)
    {
        // this is the last time we checked
        lastTick_ = osg::Timer::instance()->tick();
    }
}

// *****************************************************************************
void Shader::debug()
{
	ReferencedStateSet::debug();
	
	std::cout << "   ---------" << std::endl;

	// additional info
	if (vertexObject_.valid())
        std::cout << "   vert shader: LOADED" << std::endl;
    else
        std::cout << "   vert shader: NULL" << std::endl;

	if (fragmentObject_.valid())
		std::cout << "   frag shader: LOADED" << std::endl;
	else
        std::cout << "   frag shader: NULL" << std::endl;
    
    const osg::StateSet::UniformList uniforms = this->getUniformList();
 	osg::StateSet::UniformList::const_iterator uitr;
    for (uitr=uniforms.begin(); uitr!=uniforms.end(); ++uitr)
    {
        const std::string& name = uitr->first;
        osg::Uniform *u = uitr->second.first;
        std::cout << "   uniform " << name;
        
        float f;
        int i;
        bool b;
        osg::Vec2 v2;
        osg::Vec3 v3;
        osg::Vec4 v4;
        switch (u->getType())
        {
            case osg::Uniform::FLOAT:
                u->get(f);
                std::cout << " (float) = " << f;
                break;
            case osg::Uniform::FLOAT_VEC2:
                u->get(v2);
                std::cout << " (vec2) = " << stringify(v2);
                break;
            case osg::Uniform::FLOAT_VEC3:
                u->get(v3);
                std::cout << " (vec3) = " << stringify(v3);
                break;
            case osg::Uniform::FLOAT_VEC4:
                u->get(v4);
                std::cout << " (vec4) = " << stringify(v4);
                break;
            case osg::Uniform::INT:
                u->get(i);
                std::cout << " (int) = " << i;
                break;
            case osg::Uniform::BOOL:
                u->get(b);
                std::cout << " (bool) = " << b;
                break;
            case osg::Uniform::SAMPLER_2D:
                u->get(i);
                std::cout << " (sampler2D) = " << i;
                break;

            default:
                std::cout << " (other/unkown)";
                break;
        }
        std::cout << std::endl;
    }
    
    int numTextures = this->getTextureAttributeList().size();
    if (getTextureAttributeList().size())
    {
        std::cout << "   " << numTextures << " textures:" << std::endl;
        for (int i=0; i<numTextures; i++)
        {
            osg::StateAttribute *attr = this->getTextureAttribute(i,osg::StateAttribute::TEXTURE);
            if (attr)
            {
                std::string imageFile = attr->asTexture()->getImage(0)->getFileName();
                std::cout << "   texture " << i << ": " << imageFile << std::endl;
            }
        }
    }
    else
    {
        std::cout << "   NO textures" << std::endl;
    }
        
}

void Shader::printShaderSource()
{
	std::cout << "---------" << std::endl;
    std::cout << "VERTEX shader:" << std::endl;
	if (vertexObject_.valid())
        std::cout << vertexObject_->getShaderSource() << std::endl;
    else
        std::cout << "  N/A" << std::endl;
    std::cout << "---------" << std::endl;
    std::cout << "FRAGMENT shader:" << std::endl;
	if (fragmentObject_.valid())
        std::cout << fragmentObject_->getShaderSource() << std::endl;
    else
        std::cout << "  N/A" << std::endl;
    std::cout << "---------" << std::endl;
    
    BROADCAST(this, "s", "printShaderSource");
}  


// -----------------------------------------------------------------------------

void Shader::clearUniforms()
{
    while (this->getUniformList().size())
    {
        osg::Uniform *u = this->getUniformList().begin()->second.first;
        this->removeUniform(u);
    }
}

void Shader::registerUniform(const char* name, const char* type)
{
    registerUniform(name,type,"");
}

void Shader::registerUniform(const char* name, const char* type, const char* defaultValue)
{
    if (this->getUniform(name))
    {
        std::cout << "Uniform '" << name << "' already exists" << std::endl;
        return;
    }

    std::string typeStr = std::string(type);
    std::string defaultStr = std::string(defaultValue);

    if (!defaultStr.size())
    {
        if (typeStr=="float") defaultStr = "0.0";
        else if (typeStr=="vec2") defaultStr = "0.0 0.0";
        else if (typeStr=="vec3") defaultStr = "0.0 0.0 0.0";
        else if (typeStr=="vec4") defaultStr = "0.0 0.0 0.0 0.0";
        else defaultStr = "0";
    }

    std::vector<std::string> argVector = tokenize(defaultStr);

    float f1, f2, f3, f4;
    int i;
    bool b;
    
    bool error = false;
    if (typeStr=="float")
    {
        if ( (argVector.size()==1)
        && fromString<float>(f1, argVector[0]) )
            this->addUniform(new osg::Uniform( name, f1 ));
        else 
            error = true;
    }
    else if (typeStr=="bool")
    {
        if ( (argVector.size()==1)
        && fromString<bool>(b, argVector[0]) )
            this->addUniform(new osg::Uniform( name, b ));
        else
            error = true;
    }
    else if (typeStr=="int")
    {
        if ( (argVector.size()==1)
        && fromString<int>(i, argVector[0]) )
            this->addUniform(new osg::Uniform( name, i ));
        else
            error = true;

    }
    else if (typeStr=="vec2")
    {
        // note, args could be a multiple (eg, .jxs format). If so, let's create
        // several uniforms with array indices in the names
        if (argVector.size() < 2)
            error = true;
        else
        {
            for (int index=0; index<(int)(argVector.size()/2); index++)
            {
                std::string indexName;
                if (argVector.size()>2)
                    indexName = std::string(name)+"["+stringify(index)+"]";
                else
                    indexName = std::string(name);
                if (fromString<float>(f1, argVector[(index*2)+0])
                 && fromString<float>(f2, argVector[(index*2)+1]) )
               this->addUniform(new osg::Uniform( indexName.c_str(), osg::Vec2(f1,f2) ));
            }
        }

    }
    else if (typeStr=="vec3")
    {
        // note, args could be a multiple (eg, .jxs format). If so, let's create
        // several uniforms with array indices in the names
        if (argVector.size() < 3)
            error = true;
        else
        {
            for (int index=0; index<(int)(argVector.size()/3); index++)
            {
                std::string indexName;
                if (argVector.size()>3)
                    indexName = std::string(name)+"["+stringify(index)+"]";
                else
                    indexName = std::string(name);
                if (fromString<float>(f1, argVector[(index*3)+0])
                 && fromString<float>(f2, argVector[(index*3)+1])
                 && fromString<float>(f3, argVector[(index*3)+2]) )
               this->addUniform(new osg::Uniform( indexName.c_str(), osg::Vec3(f1,f2,f3) ));
            }
        }
    }
    else if (typeStr=="vec4")
    {
        // note, args could be a multiple (eg, .jxs format). If so, let's create
        // several uniforms with array indices in the names
        if (argVector.size() < 4)
            error = true;
        else
        {
            for (int index=0; index<(int)(argVector.size()/4); index++)
            {
                std::string indexName;
                if (argVector.size()>4)
                    indexName = std::string(name)+"["+stringify(index)+"]";
                else
                    indexName = std::string(name);
                if (fromString<float>(f1, argVector[(index*4)+0])
                 && fromString<float>(f2, argVector[(index*4)+1])
                 && fromString<float>(f3, argVector[(index*4)+2])
                 && fromString<float>(f4, argVector[(index*4)+3]) )
               this->addUniform(new osg::Uniform( indexName.c_str(), osg::Vec4(f1,f2,f3,f4) ));
            }
        }
    }
    else if (typeStr=="sampler2DRect")
    {
        if ( (argVector.size()==1)
        && fromString<int>(i, argVector[0]) )
            this->addUniform(new osg::Uniform( osg::Uniform::SAMPLER_2D, name, i ));
        else
            error = true;
    }
    else if (typeStr=="sampler2D")
    {
        if ( (argVector.size()==1)
        && fromString<int>(i, argVector[0]) )
            this->addUniform(new osg::Uniform( osg::Uniform::SAMPLER_2D, name, i ));
        else
            error = true;
    }
    else
    {
        error = true;
    }
    
    
    // output a warning if we couldn't create the uniform, otherwise if this is
    // the server, send the message to clients:
    if (error)
    {
        std::cout << "Warning: Could not create uniform '" << name << "'. Perhaps  invalid type: '" << type << "', or incorrect args: {" << defaultStr << "}" << std::endl;
    }
    else
    {
        std::cout << "Registered uniform '" << name << "' (" << type << ") with value: " << defaultStr << std::endl;
        
        BROADCAST(this, "ssss", "registerUniform", name, type, defaultStr.c_str());
    }
}

bool Shader::loadJitterShader(std::string path)
{
    TiXmlDocument doc( path.c_str() );

    TiXmlNode *root = 0;
    TiXmlElement *child = 0;
    TiXmlElement *child2 = 0;

    osg::Shader *vertShader = 0;
    osg::Shader *fragShader = 0;

    // Load the XML file and verify:
    if (! doc.LoadFile())
    {
        std::cout << "ERROR: failed to load " << path << ". Invalid XML format." << std::endl;
        return false;
    }

    // get the <jittershader> tag and verify:
    if (! (root = doc.FirstChild("jittershader")))
    {
        std::cout << "ERROR: failed to load " << path << ". XML file has no <jittershader> tag." << std::endl;
        return false;
    }

    //if (root->Attribute("name"))
    //    programObject_->setName(root->Attribute("name"));

    child = root->FirstChildElement("language");
    if (child && (std::string(child->Attribute("name"))=="glsl"))
    {
        // look for shader programs:
        for (child2 = child->FirstChildElement("program"); child2; child2 = child2->NextSiblingElement())
        {
            if (std::string(child2->Attribute("type"))=="vertex")
            {
                vertexObject_ = new osg::Shader( osg::Shader::VERTEX );
                if (child2->Attribute("source"))
                {
                    if (loadShaderSource(vertexObject_, osgDB::getFilePath(path)+"/"+child2->Attribute("source")))
                        programObject_->addShader( vertexObject_ );
                }
                else
                {
                    vertexObject_->setShaderSource(child2->FirstChild()->Value());
                    replaceSampler2DRect(vertexObject_);
                    programObject_->addShader( vertexObject_ );
                }
            }
            else if (std::string(child2->Attribute("type"))=="fragment")
            {
                fragmentObject_ = new osg::Shader( osg::Shader::FRAGMENT );
                if (child2->Attribute("source"))
                {
                    if (loadShaderSource(fragmentObject_, osgDB::getFilePath(path)+"/"+child2->Attribute("source")))
                        programObject_->addShader( fragmentObject_ );
                }
                else
                {
                    fragmentObject_->setShaderSource(child2->FirstChild()->Value());
                    replaceSampler2DRect(fragmentObject_);
                    programObject_->addShader( fragmentObject_ );
                }
                
                // Look for sampler2Drect, because sometimes the shader author
                // doesn't include it in the .jxs file, even though it is used.
                ParsedUniforms uniforms;
                uniforms = parseUniformsFromShader(vertexObject_.get());
                for (ParsedUniforms::iterator u=uniforms.begin(); u!=uniforms.end(); ++u)
                {
                    if (u->second=="sampler2DRect")
                    {
                        registerUniform(u->first.c_str(), u->second.c_str());
                    }
                }
                uniforms.clear();
                uniforms = parseUniformsFromShader(fragmentObject_.get());
                for (ParsedUniforms::iterator u=uniforms.begin(); u!=uniforms.end(); ++u)
                {
                    if (u->second=="sampler2DRect")
                    {
                        registerUniform(u->first.c_str(), u->second.c_str());
                    }
                }
            }
        } 
    }
    else
    {
        std::cout << "ERROR: the shader " << path << " is not GLSL. Only GLSL is supported." << std::endl;
        return false;
    }
    
    // load all uniforms:
    for (child = root->FirstChildElement("param"); child; child = child->NextSiblingElement())
    {
        // uniform
        if ( (child->Attribute("name")) 
        && (child->Attribute("type")) 
        && (child->Attribute("default")) )
        {
            registerUniform(child->Attribute("name"), child->Attribute("type"), child->Attribute("default"));
        }
    }
    
    return true;
}

void Shader::loadGLSLShader(std::string path)
{
    // create tmp shaders:
    osg::Shader *vertShader = new osg::Shader( osg::Shader::VERTEX );
    osg::Shader *fragShader = new osg::Shader( osg::Shader::FRAGMENT );

    std::string folderPath = osgDB::getFilePath(path);

    bool vertSuccess = false;
    bool fragSuccess = false;
    
    std::string ext = osgDB::getLowerCaseFileExtension(path);
    if (ext=="frag")
    {
        fragSuccess = loadShaderSource( fragShader, path );
        vertSuccess = loadShaderSource( vertShader, folderPath+".vert" );
    }
    else if (ext=="vert")
    {
        vertSuccess = loadShaderSource( vertShader, path );
        fragSuccess = loadShaderSource( fragShader, folderPath+".frag" );
    }
    else
    {
        // let's check if the user just passed the base file name (without an extention)
        vertSuccess = loadShaderSource( vertShader, path+".vert" );
        fragSuccess = loadShaderSource( fragShader, path+".frag" );
    }

    if (vertSuccess)
    {
        vertexObject_ = vertShader;
        
        ParsedUniforms uniforms = parseUniformsFromShader(vertexObject_.get());
        for (ParsedUniforms::iterator u=uniforms.begin(); u!=uniforms.end(); ++u)
            registerUniform(u->first.c_str(), u->second.c_str());

        programObject_->addShader( vertexObject_ );
    }
    if (fragSuccess)
    {
        fragmentObject_ = fragShader;
        
        ParsedUniforms uniforms = parseUniformsFromShader(fragmentObject_.get());
        for (ParsedUniforms::iterator u=uniforms.begin(); u!=uniforms.end(); ++u)
            registerUniform(u->first.c_str(), u->second.c_str());

        programObject_->addShader( fragmentObject_ );
    }
    
    programObject_->setName(osgDB::getStrippedName(path));
}

// *****************************************************************************
void Shader::setUniform_bool (const char* name, int b)
{
    osg::Uniform *u = this->getUniform(name);
    if (u) u->set((bool)b);
    
    BROADCAST(this, "ssb", "setUniform_bool", name, (bool)b);
}

void Shader::setUniform_int (const char* name, int i)
{
    osg::Uniform *u = this->getUniform(name);
    if (u) u->set(i);
    
    BROADCAST(this, "ssi", "setUniform_int", name, i);
}

void Shader::setUniform_float (const char* name, float f)
{
    osg::Uniform *u = this->getUniform(name);
    if (u) u->set(f);
    
    BROADCAST(this, "ssf", "setUniform_float", name, f);
}

void Shader::setUniform_vec2 (const char* name, float x, float y)
{
    osg::Vec2 v = osg::Vec2(x,y);
    osg::Uniform *u = this->getUniform(name);
    if (u) u->set(v);
    
    BROADCAST(this, "ssff", "setUniform_vec2", name, x, y);
}

void Shader::setUniform_vec3 (const char* name, float x, float y, float z)
{
    osg::Vec3 v = osg::Vec3(x,y,z);
    osg::Uniform *u = this->getUniform(name);
    if (u) u->set(v);
    
    BROADCAST(this, "ssfff", "setUniform_vec3", name, x, y, z);
}

void Shader::setUniform_vec4 (const char* name, float x, float y, float z, float w)
{
    osg::Vec4 v = osg::Vec4(x,y,z,w);
    osg::Uniform *u = this->getUniform(name);
    if (u) u->set(v);
    
    BROADCAST(this, "ssffff", "setUniform_vec4", name, x, y, z, w);
}

// *****************************************************************************
void Shader::setShader (const char* path)
{
	// only do this if the id has changed:
	if (shaderPath_ == std::string(path)) return;

	shaderPath_ = std::string(path);
    
    // remove previous shader:
    this->removeAttribute(programObject_);
    if (vertexObject_)
    {
        programObject_->removeShader(vertexObject_);
        vertexObject_ = 0;
    }
    if (fragmentObject_)
    {
        programObject_->removeShader(fragmentObject_);
        fragmentObject_ = 0;
    }
    clearUniforms();
    
    
    if (shaderPath_ == "NULL")
    {
        std::cout << "Disabled shader '"<< this->id->s_name << "'" << std::endl;
    }
    else
    {
        // create new shaders:
        std::string fullPath = getAbsolutePath(shaderPath_);
        std::cout << "Loading shader: " << fullPath << std::endl;

        std::string ext = osgDB::getLowerCaseFileExtension(fullPath);
        if (ext=="jxs")
        {
            loadJitterShader(fullPath);
        } else {
            loadGLSLShader(fullPath);
        }
        
        // enable shader:
        if (vertexObject_.valid() && fragmentObject_.valid())
        {
            this->setAttributeAndModes(programObject_);
            programObject_->dirtyProgram();
        } else {
            std::cout << "ERROR loading shader from path: " << shaderPath_ << std::endl;
        }
	}

	BROADCAST(this, "ss", "setShader", getShader());
}

// *****************************************************************************
std::vector<lo_message> Shader::getState () const
{
	// inherit state from base class
	std::vector<lo_message> ret = ReferencedStateSet::getState();
		
	lo_message msg;
	
	msg = lo_message_new();
	lo_message_add(msg, "ss", "setShader", getShader());
	ret.push_back(msg);
	
    // send/save all current uniform values:
    
    const osg::StateSet::UniformList uniforms = this->getUniformList();
 	osg::StateSet::UniformList::const_iterator uitr;
    for (uitr=uniforms.begin(); uitr!=uniforms.end(); ++uitr)
    {
        msg = lo_message_new();
    
        const std::string& name = uitr->first;
        osg::Uniform *u = uitr->second.first;
        
        float f;
        int i;
        bool b;
        osg::Vec2 v2;
        osg::Vec3 v3;
        osg::Vec4 v4;
        switch (u->getType())
        {
            case osg::Uniform::FLOAT:
                u->get(f);
                lo_message_add(msg, "ssf", "setUniform_float", name.c_str(), f);
                break;
            case osg::Uniform::FLOAT_VEC2:
                u->get(v2);
                lo_message_add(msg, "ssff", "setUniform_vec2", name.c_str(), v2.x(), v2.y());
                break;
            case osg::Uniform::FLOAT_VEC3:
                u->get(v3);
                lo_message_add(msg, "ssfff", "setUniform_vec3", name.c_str(), v3.x(), v3.y(), v3.z());
                break;
            case osg::Uniform::FLOAT_VEC4:
                u->get(v4);
                lo_message_add(msg, "ssffff", "setUniform_vec4", name.c_str(), v4.x(), v4.y(), v4.z(), v4.w());
                break;
            case osg::Uniform::INT:
                u->get(i);
                lo_message_add(msg, "ssi", "setUniform_int", name.c_str(), i);
                break;
            case osg::Uniform::BOOL:
                u->get(b);
                lo_message_add(msg, "ssi", "setUniform_bool", name.c_str(), int(b));
                break;
            case osg::Uniform::SAMPLER_2D:
                u->get(i);
                lo_message_add(msg, "ssi", "setUniform_int", name.c_str(), i);
                break;

            default:
                break;
        }
        
        ret.push_back(msg);
    }

    
    
    
    
	return ret;
}

} // end of namespace spin

