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

#ifndef __Shader_H
#define __Shader_H

class SceneManager;

#include <osg/Timer>
#include <osgDB/FileUtils>
namespace osg {
    class Program;
    class Shader;
    class Uniform;
}

#include <string>
#include <lo/lo_types.h>
#include <vector>
#include <osg/ref_ptr>
#include "referencedstateset.h"
#include "shaderutil.h"


namespace spin
{

/**
 * \brief A wrapper for GLSL shaders
 *
 */
class Shader : public ReferencedStateSet
{

public:

    Shader(SceneManager *sceneManager, const char *initID);
    ~Shader();

    virtual void updateCallback();
    virtual void debug();
    
    void printShaderSource();
    
    /**
     * Remove all uniforms from this stateset
     */
    void clearUniforms();
    
    /**
     * Create a uniform with the provided name and type. The value will be set
     * to some default value
     */ 
    void registerUniform(const char* name, const char* type);
    
    /**
     * Create a uniform with the provided name and type, and set the initial 
     * value for the uniform.
     *
     * Note that defaults are provided as strings in all cases. So in the case
     * of several values, the defaultValue would be a space-separated list of
     * values.
     *
     * It is also possible to specify an array of vectors. For example, if the
     * type is vec2, you can specify a defaultValue of 0.0 0.1 1.0 1.0 and this
     * will generate two vec2 uniforms with indexes automatically added to the
     * name:
     *   uniform name[0] = vec2(0.0,0.1);
     *   uniform name[1] = vec2(1.0,1.0);
     * To send OSC messages to control these uniforms, you need to use the full
     * name including square brackets. For example:
     *   /SPIN/default/shader setUniform_vec2 name[0] 0.0 0.0
     */
    void registerUniform(const char* name, const char* type, const char* defaultValue);
    
    bool loadJitterShader(std::string path);
    void loadGLSLShader(std::string path);

    void setUniform_bool (const char* name, int b);
    void setUniform_int (const char* name, int b);
    void setUniform_float (const char* name, float f);
    void setUniform_vec2 (const char* name, float x, float y);
    void setUniform_vec3 (const char* name, float x, float y, float z);
    void setUniform_vec4 (const char* name, float x, float y, float z, float w);

    /**
     * Creates a texture from a path on disk.
     */
    void setShader (const char* path);
    const char *getShader() const { return shaderPath_.c_str(); }

    // must reimplement
    virtual std::vector<lo_message> getState() const;

    
protected:
    
    osg::ref_ptr<osg::Program> programObject_;
    osg::ref_ptr<osg::Shader> vertexObject_;
    osg::ref_ptr<osg::Shader> geometryObject_;
    osg::ref_ptr<osg::Shader> fragmentObject_;
    
    std::string shaderPath_;
    
    float updateRate_; // seconds
    
    osg::Timer_t lastTick_;

};

} // end of namespace spin

#endif
