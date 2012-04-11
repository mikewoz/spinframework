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

#ifndef ASMANIPULATOR_H_
#define ASMANIPULATOR_H_

#include <osgGA/GUIActionAdapter>
#include <osgManipulator/Dragger> // for PointerInf
#include <osgManipulator/Selection> // for Selection (typedef)
#include <osgUtil/LineSegmentIntersector>
#include "RayNode.h"


namespace osgGA {
    class GUIEventAdapter;
}

namespace osgManipulator {
    class CommandManager;
}

namespace spin
{

/**
 * Used internally by PointerNode. Dummy class.
 */
class PointerNodeActionAdapter : public osgGA::GUIActionAdapter
{
    void requestRedraw(void){};
    void requestContinuousUpdate(bool){};
    void requestWarpPointer(float,float) {};
};
  
/**
 * \brief An interaction node that reports intersections with other nodes in the
 *        scene (test)
 * 
 * This node must be attached to an RayNode node, and reports a list of all nodes
 * with which the ray is intersecting (in order of closest to furthest). There
 * is also a grabber manipulator that allows the first node to be "grabbed" and
 * moved around.
 */

class PointerNode : public RayNode
{

    public:

        PointerNode(SceneManager *sceneManager, char *initID);
        virtual ~PointerNode();

        virtual void callbackUpdate();


        ReferencedNode *getNodeFromIntersections();
        //void computeRT(t_symbol *src, t_symbol *dst, osg::Vec3 &R, osg::Vec3 &T);


        void manipulate (int b);
        int getManipulate() const { return (int) doManipulation; }

        /**
         * The grab method selects the closest intersected node and temporarily
         * attaches it to the pointer, allowing it to inherit any translation or
         * rotation offsets.
         *
         * Notes:
         * - Only nodes derived from GroupNode can be grabbed.
         * - If no node is intersected, the grab won't do anything.
         * - The node is re-attached to it's original parent when released, so
         * don't delete the parent in the meantime
         * 
         * @param b A boolean grab indicator (1 to grab, 0 to release)
         */
        void grab (int b);
    
        /**
         * Slides the currently grabbed node (if there is one) along the pointer
         * axis (ie, increasing or decreasing the distance).
         *
         * @param f The amount by which to slide (positive values slide the 
         * attached node AWAY from the pointer
         */
        void slide (float f);
         
        /**
         * @return Whether there is a valid node that is currently 'grabbed'
         */
        int getGrab() const { return (int) grabbedNode.valid(); }

        /**
         * For each subclass of ReferencedNode, we override the getState() method to
         * fill the vector with the correct set of methods for this particular node
         */
        virtual std::vector<lo_message> getState() const;

    protected:
        /**
         * Checks intersections for draggers and if so, we apply the drag events
         */
        void applyManipulation(osg::Matrix mat, osg::Vec3 start, osg::Vec3 end);

        /**
         * Reports the list of intersected nodes (server-side only)
         */
        void reportIntersections();

    private:

        // intersector stuff:
        osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector;
        std::vector<t_symbol*> intersectList;
        std::vector<osg::Vec3> intersectListOffsets;
        std::vector<osgUtil::LineSegmentIntersector::Intersection> intersectData;

        // grabber stuff:
        osg::ref_ptr<ReferencedNode> grabbedNode;
        osg::Vec3 grabbedOffset;
        t_symbol *previousParent;

        // dragger stuff:
        bool doManipulation;
        osg::ref_ptr<ReferencedNode> targetNode;

        osg::Matrix origMatrix;
        osg::Matrix previousMatrix;
        
        /*
        osg::ref_ptr<osgManipulator::Dragger> dragger;
        osg::ref_ptr<osgManipulator::Selection> selection;
        osg::ref_ptr<osgManipulator::CommandManager> cmdMgr;
        */
        
        osg::ref_ptr<osgGA::GUIEventAdapter> ea;
        PointerNodeActionAdapter aa;
        
        osgManipulator::PointerInfo _pointer;
};

} // end of namespace spin

#endif
