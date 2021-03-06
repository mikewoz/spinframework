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

#ifndef __SoundNode_H
#define __SoundNode_H

#include "dspnode.h"

namespace spatosc
{
    class SoundSource;
}

namespace spin
{

/**
 * \brief Represents a point sound source (or sink).
 * 
 * The SoundNode class allows for the positioning of a sound node in 3D space,
 * and controlling various aspects such as directivity and visual rendering for
 * debugging purposes.
 */
class SoundNode : public DSPNode
{
    
    public:
        
        SoundNode(SceneManager *sceneManager, const char* initID);
        virtual ~SoundNode();

        virtual void debug();
        virtual void callbackUpdate(osg::NodeVisitor* nv);
        bool dumpGlobals(bool forced=false);
        
        // override some methods so that we can send them to SpatOSC:
        virtual void setParam (const char *paramName, const char *paramValue);
        virtual void setParam (const char *paramName, float paramValue);
        virtual void setTranslation (float x, float y, float z);
        virtual void setOrientation (float p, float r, float y);
        virtual void setOrientationQuat (float x, float y, float z, float w);
        virtual void setRadius (float f);
        virtual void setActive (int f);
        virtual void setTransitionFactor (float f);
        virtual void setURI (const char *uri);
        virtual void setDirectivity(const char* horizPattern, const char* vertPattern);
        virtual void connect (const char* sinkNodeID);
        virtual void disconnect (const char* sinkNodeID);
        virtual void setConnectionParam (const char* sinkNodeID, const char* method, float value);
        virtual void sendEvent (const char *types, lo_arg **argv, int argc );

        /**
         * For each subclass of ReferencedNode, we override the getState()
         * method to fill the vector with the correct set of methods for this
         * particular node
         */
        virtual std::vector<lo_message> getState() const;
        
    private:
        
#ifdef WITH_SPATOSC
        spatosc::SoundSource *spatOSCSource;
#endif

};

} // end of namespace spin


#endif
