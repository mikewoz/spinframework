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
namespace osg {
    class Program;
    class Shader;
    class Uniform;
}

#include <string>
#include <lo/lo_types.h>
#include <vector>
#include <osg/ref_ptr>
#include "ReferencedStateSet.h"

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

    void setUniform_Float (const char* name, float f);
    void setUniform_Vec3 (const char* name, float x, float y, float z);

    /**
     * Creates a texture from a path on disk.
     */
    void setPath (const char* newPath);
    const char *getPath() const { return _path.c_str(); }

    /**
     * Set whether the texture is influenced by lighting
     */
    void setLighting(int i);
    int getLighting() const { return (int)_lightingEnabled; }

    /**
     * Set the render bin for this texture. The higher the number, the later it
     * gets processed (ie, it appears on top). Default renderBin = 11
     */
    void setRenderBin (int i);
    int getRenderBin() const { return _renderBin; }



    // must reimplement
    virtual std::vector<lo_message> getState() const;

    
private:
    
    osg::ref_ptr<osg::Program> _programObject;
    osg::ref_ptr<osg::Shader> _vertexObject;
    osg::ref_ptr<osg::Shader> _fragmentObject;
    
    std::string _path;
    
    float _updateRate; // seconds
    
    osg::Timer_t lastTick;

    bool _lightingEnabled;
    int  _renderBin;
};

} // end of namespace spin

#endif
