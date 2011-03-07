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

#ifndef Contour_H_
#define Contour_H_

namespace osg {
    class Geode;
    class PositionAttitudeTransform;
}

#include "ReferencedNode.h"

enum ContourTypeEnum { THIN, CUBIC, CYLINDRICAL };
enum trackingModeEnum { POSITION, FULL6DOF };

/**
 * \brief Represents a sequence of connected points in 3D space
 * 
 * The contour holds a number of vertices up the limit specified by _maxVertices.
 * New values are added to the front of the vectorArray, and once it is filled,
 * we pop the last element off the end.
 * 
 * An index controls the point on which a child node is attached to the contour.
 */
class Contour : public ReferencedNode
{

public:

    Contour(SceneManager *sceneManager, char *initID);
    virtual ~Contour();
        
    /**
     * IMPORTANT:
     * subclasses of ReferencedNode are allowed to contain complicated subgraphs,
     * and can also change their attachmentNode so that children are attached
     * anywhere in this subgraph. If that is the case, the updateNodePath()
     * function MUST be overridden, and extra nodes must be manually pushed onto
     * the currentNodePath.
     */
    virtual void updateNodePath();

    virtual void callbackUpdate();
    
    void updateTransforms();
    osg::Quat getOrientation(int index) const;
    osg::Vec3 getTranslation(float index) const;
    
    void setCurrentIndex (float newValue);
    void prev();
    void next();

    void reset();
    
    void add (float x, float y, float z);
    void setMaxVertices (int newValue);
    
    void setTrackingMode(int newValue);
    
    void setVisible (int newValue);
    void setThickness (float newValue);
    void setLineType (int newValue);
    void setColor (float newR, float newG, float newB, float newA);
        
    
    float getCurrentIndex() const { return _currentIndex; }
    int getMaxVertices() const { return _maxVertices; }
    
    int getTrackingMode() const { return (int) _trackingMode; }
    
    int getVisible() const { return (int) _visible; }
    float getThickness() const { return _thickness; }
    float getLineType() const { return (int) _lineType; }
    osg::Vec4 getColor() const { return _color;  }
    
    
    
    void draw();
    

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
        
    // ============

private:
    
    bool _visible;
    
    bool _redrawFlag;
    
    osg::ref_ptr<osg::Vec3Array> _vArray;
    
    float _currentIndex; // allow float indices for interpolation between values
    int _maxVertices;

    trackingModeEnum _trackingMode;

    
    osg::Vec4 _color;
    
    ContourTypeEnum _lineType;
    float _thickness;

    osg::ref_ptr<osg::PositionAttitudeTransform> mainTransform;
    osg::ref_ptr<osg::Geode> vArrayGeode;
    //osg::ref_ptr<osg::Geometry> vArrayGeometry;

};


#endif
