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

#ifndef LightSource_H_
#define LightSource_H_

#include "GroupNode.h"

namespace osg {
    class LightSource;
}

/**
 * \brief A light source, with controllable intensity, color, etc.
 *
 * Note that only 8 light sources are available.
 */
class LightSource : public GroupNode
{

public:

    LightSource(SceneManager *sceneManager, char *initID);
    virtual ~LightSource();

    void setVisible        (int visibilityFlag);
    void setCutoff        (float cutoff);
    void setExponent    (float exponent);
    void setAttenuation    (float attenuation);

    void setAmbient        (float red, float green, float blue, float alpha);
    void setDiffuse        (float red, float green, float blue, float alpha);
    void setSpecular    (float red, float green, float blue, float alpha);

    int getVisible() const;
    float getCutoff() const;
    float getExponent() const;
    float getAttenuation() const;
    osg::Vec4 getAmbient() const;
    osg::Vec4 getDiffuse() const;
    osg::Vec4 getSpecular() const;

    /**
     * For each subclass of ReferencedNode, we override the getState() method to
     * fill the vector with the correct set of methods for this particular node
     */
    virtual std::vector<lo_message> getState() const;

    /**
     * We must include a stateDump() method that simply invokes the base class
     * method. Simple C++ inheritance is not enough, because osg::Introspection
     * won't see it.
     */
    //virtual void stateDump() { ReferencedNode::stateDump(); };


private:

    void drawLight();

    int lightNum;
    osg::ref_ptr<osg::LightSource> lightSource;

    bool _visible;
    float _cutoff;
    float _exponent;
    float _attenuation;

    // lighting color parameters:
    osg::Vec4 _ambient;
    osg::Vec4 _diffuse;
    osg::Vec4 _specular;

};



#endif
