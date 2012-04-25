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

#ifndef __ReferencedStateSet_H
#define __ReferencedStateSet_H

#include "spinUtil.h"
#include "libloUtil.h"

#ifndef GL_GLEXT_LEGACY
#define GL_GLEXT_LEGACY // To avoid glext error
#endif
#include <osg/StateSet>
#include <osg/TexEnv>

namespace spin
{

// forward declaration of SceneManager
class SceneManager;


/**
 * \brief ReferencedStateSet is the base class for StateSets attached to nodes
 */

class ReferencedStateSet : virtual public osg::StateSet
{

public:

    ReferencedStateSet(SceneManager *sceneManager, const char *initID);
    ~ReferencedStateSet();

    /**
     * This callback occurs every frame to update the state with any parameter
     * changes.
     */

    virtual void updateCallback();

    /**
     * Abstract method getPath needs to be implemented
     */
    //virtual const char *getPath() const = 0;
	
    /**
    * Remove this stateset from all parents... essentially destroying the state,
    * since no reference to it will exist anymore, and OSG will kill it.
    */
    void removeFromScene();

    /**
     * Replaces a StateSet in the scene graph with this one. ie, goes through
     * all parents of the provided stateset and replaces the object's state with
     * this.
     */ 
    void replace(osg::StateSet *ss);
    
    /**
     * Print debug information to console.
     */

    virtual void debug();
    
    /**
     * Just like a ReferencedNode, each subclass of ReferencedStateSet must
     * override the getState() method to pass it's current state.
     */
    virtual std::vector<lo_message> getState() const;

    /**
     * StateDump() is a request to broadcast the node state via SceneManager.
     */
    virtual void stateDump();

    /**
     * StateDump() is a request to broadcast the node state via SceneMangager.
     */

    virtual void stateDump(lo_address txAddr);

	// --------------------------------

    /**
     * Set the blend mode for the texture (with material color).
	 * 0=GL_MODULATE, 1=GL_DECAL, 2=GL_BLEND, 3=GL_REPLACE.
	 * Default is 0 (GL_MODULATE)
     */
    virtual void setTextureBlend(int mode);
	int getTextureBlend() const;
	
    /**
     * Set whether the texture repeats after wrapping or not (in both the x and
     * Y directions
     */
    virtual void setTextureRepeat(int s, int t);
	
    /**
     * Set whether the texture is influenced by lighting
     */
    virtual void setLighting(int i);

    /**
     * Returns a boolean indicating whether lighting affects the texture.
     */

    virtual int getLighting() const { return (int)lightingEnabled_; }

    /**
     * Set the render bin for this texture. The higher the number, the later it
     * gets processed (ie, it appears on top). Default renderBin = 11
     */
    virtual void setRenderBin (int i);

    /**
     * Returns an integer indicating the render bin for this texture. Higher
     * numbers get processed later (i.e. it appears on top). Default = 11
     */
    virtual int getRenderBin() const { return renderBin_; }

	// TODO: these should at least be protected:
    t_symbol *id;
    std::string classType;

protected:
    
	osg::TexEnv::Mode textureBlend_;
	bool textureRepeatS_;
	bool textureRepeatT_;
	bool lightingEnabled_;
    int  renderBin_;

    SceneManager *sceneManager;

};

typedef std::vector< osg::ref_ptr<ReferencedStateSet> > stateListType;

class ReferencedStateSet_callback : public osg::StateSet::StateSet::Callback
{
    public:
        
        virtual void operator()(osg::StateSet* ss, osg::NodeVisitor* /*nv*/)
        {
            osg::ref_ptr<ReferencedStateSet> thisState = dynamic_cast<ReferencedStateSet*> (ss->getUserData());

            if (thisState.valid())
            {
                thisState->updateCallback();
            }
        }
};

} // end of namespace spin


#endif
