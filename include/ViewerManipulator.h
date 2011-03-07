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

#ifndef __ViewerManipulator_H
#define __ViewerManipulator_H

#include <osg/ref_ptr>
#include <osgGA/NodeTrackerManipulator>
#include <cstdarg>
#include "spinUtil.h" // for t_symbol

// forward declarations
class GroupNode;

namespace osgViewer {
    class View;
}

/**
 * \brief This class provides camera control and picking for viewers that render
 *        a SPIN scene.
 *
 * We override the osgGA::NodeTrackerManipulator class so that our camera can
 * follow any node (ie, the UserNode for the viewer). However, mouse events are
 * handled differently. Instead of directly affecting the state of the viewer's
 * graph, we send the events to the server, which will update the UserNode and
 * the changes will be updated on ALL machines.
 * 
 * Additionally, picking of interactive nodes is supported, allowing the viewer
 * to manipulate content of a SPIN scene using the mouse.
 */

class ViewerManipulator : public osgGA::NodeTrackerManipulator
{
    public:
        //ViewerManipulator(spinContext *s, UserNode *u);
        //ViewerManipulator(UserNode *u);
        ViewerManipulator();
        
        void setPicker(bool b);
        void setMover(bool b);
        void setRaw(bool b);
        
        bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa);
        void handleKeypress(const osgGA::GUIEventAdapter& ea);
        void handleMouse(osgViewer::View* view, const osgGA::GUIEventAdapter& ea);
        
        //GroupNode* getNodeFromIntersection(osgUtil::LineSegmentIntersector::Intersection intersection);
        //std::vector<GroupNode*> getNodesFromIntersections(osgUtil::LineSegmentIntersector::Intersections intersections);

        void sendPick(osg::ref_ptr<GroupNode> hitNode, unsigned int eventType, unsigned int modKeyMask, unsigned int buttonMask, float scrollX, float scrollY, float dX, float dY, osg::Vec3 hitPoint);
        void sendEvent(const char *nodeId, const char *types, ...);
        void sendEvent(const char *nodeId, const char *types, va_list ap);


    protected:
        virtual ~ViewerManipulator();
        
        //spinContext *spin;
        //osg::ref_ptr<UserNode> user;
        t_symbol *user;
        
        //t_symbol *selectedNode;
        std::vector<t_symbol*> selectedNodes;
        
        bool picker, mover, raw;
        float lastX, lastY;
        float clickX, clickY;
        
        osg::Vec3 lastHitPoint;
};


#endif
