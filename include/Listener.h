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

#ifndef __Listener_H
#define __Listener_H

#include "DSPNode.h"

namespace spatosc
{
    class Listener;
}

namespace spin
{

/**
 * \brief Represents an audio listener in 3D.
 * 
 * The Listener class is a special type of SoundNode, that allows for different
 * types of connections. 
 */
class Listener : public DSPNode
{
    public:
        Listener(SceneManager *sceneManager, const char* initID);
        virtual ~Listener();
        
        virtual void callbackUpdate(osg::NodeVisitor* nv);
        bool dumpGlobals(bool forced=false);

        /**
         * For each subclass of ReferencedNode, we override the getState()
         * method to fill the vector with the correct set of methods for this
         * particular node
         */
        virtual std::vector<lo_message> getState() const;
        
        // override some methods so that we can send them to SpatOSC:
        virtual void setParam (const char *paramName, const char *paramValue);
        virtual void setParam (const char *paramName, float paramValue);

        virtual void setURI (const char *uri);
        
    private:

#ifdef WITH_SPATOSC
        spatosc::Listener *spatOSCListener;
#endif
};

} // end of namespace spin

#endif
