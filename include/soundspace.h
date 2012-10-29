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

#ifndef __SoundSpace_H
#define __SoundSpace_H

#ifdef AS_GRAPHICAL
#include <osgUtil/IntersectVisitor>
#endif

#include "DSPNode.h"

namespace spin
{

/**
 * \brief Represents an acoustic enclosure or volume in space.
 * 
 * The SoundSpace class allows for the creation of an acoustic space inhabited
 * by audio processing (or playback).
 */
class SoundSpace : public DSPNode
{
    
  public:
        
        SoundSpace(SceneManager *sceneManager, const char* initID);
        virtual ~SoundSpace();


        void setAbsorption (t_floatarg newval);
        void setFilterCoef (t_floatarg newval);
        void setTransition (t_floatarg newval);
        t_float absorption;
        t_float filterCoef;
        t_float transition;
        
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

#ifdef AS_GRAPHICAL
        osgUtil::IntersectVisitor soundSpaceIntersectVisitor;
#endif
    
};

} // end of namespace spin

#endif
