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
//#include "GroupNode.h"

namespace spin
{

/**
 * The ContourType enumerator contains the three types of possible contours.
 */

enum ContourTypeEnum { THIN, 		/*!< Represents a line or "snake-like"
									contour  */

						CUBIC,		/*!< Represents a cubic or rectangular
									contour */

						CYLINDRICAL	/*!< Represents a cylindrical contour */
					};

/**
 * The trackingMode enumerator determines whether children node will rotate or
 * move without rotation while moved along the contour.
 */


enum trackingModeEnum { POSITION,	/*!< Children nodes will move along the
									contour	without rotation */

						FULL6DOF	/*!< Children nodes will move along the
									contour with full rotation based on their
									attachment to the contour */
						};

/**
 * \brief Represents a sequence of connected points in 3D space
 * 
 * The contour holds a number of vertices up the limit specified by _maxVertices
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
     * subclasses of ReferencedNode are allowed to contain complicated subgraphs
     * and can also change their attachmentNode so that children are attached
     * anywhere in this subgraph. If that is the case, the updateNodePath()
     * function MUST be overridden, and extra nodes must be manually pushed onto
     * the currentNodePath.
     */

    virtual void updateNodePath();

    virtual void callbackUpdate(osg::NodeVisitor* nv);

    /**
     * Updates the contour with respect to any new parameters recently set. This
     * function is called automatically by most of the other functions, such as
     * setCurrentIndex, add, etc.
     */

    void updateTransforms();
    
    /**
    * This function gets the orientation of a given index into the soundLine.
    * Note that the orientation along a line has only 2 degrees of freedom
    * (roll is not defined). So we specify the Y_AXIS.
    */

    osg::Quat getOrientation(float index) const;

    /** We allow the index to be a float, so position can be interpolated between
    * two indices:
	*/

    osg::Vec3 getTranslation(float index) const;
    
    /**
     * Sets the "index" vertex where child nodes will be attached to the contour
     * This is handled using a value in the range 0 to 1.
     */

    void setIndex (float newValue);

    /**
     * Sets the index to the vertex previous to the current index in the contour
     */

    void prev();

    /**
     * Sets the index to the vertex after the current index in the contour.
     */

    void next();

    /**
     * Clears all information from the contour, leaving an empty node.
     */

    void reset();
    
    /**
     * Add a vertex to the front of the contour the (vectorArray). If the list
     * of vectors reaches maximum size, the final vector is removed from the
     * end of the contour. Please note that the index vertex (where children
     * nodes are attached) remains in the same position on the contour even
     * as the vertices change because this value is set as a proportion of the
     * vertex length (a 0,1 range value).
     */

    void add (float x, float y, float z);

    /**
     * Sets the maximum number of vertices on the contour.
     */

    void setMaxVertices (int newValue);
    
    /**
     * Sets the mode of the contour according to the trackingMode enumerator.
     * Determines whether children node will rotate as they move along the
     * contour (FULL6D0F) or move along the contour without rotation (POSITION).
     */

    void setTrackingMode(int newValue);
    
    /**
     * Sets whether the contour is visible or not.
     */

    void setVisible (int newValue);

    /**
     * Sets the thickness of the contour.
     */

    void setThickness (float newValue);

    /**
     * Sets the contour type according to the contourType enumerator.
     * Determines whether the connections between vertices on the contour
     * will be drawn as line segments (THIN), boxes (CUBIC), or circles
     * (CYLINDRICAL).
     */

    void setLineType (int newValue);

    /**
     * Sets the color of the contour in RGBA value.
     */

    void setColor (float newR, float newG, float newB, float newA);
    
    /**
     * Returns a float indicating the current index vertex for the contour. A
     * float will interpolate a point between vertices to act as the index.
     */

    float getCurrentIndex() const { return _currentIndex; }

    /**
     * Returns the maximum number of vertices currently permitted for the
     * contour.
     */

    int getMaxVertices() const { return _maxVertices; }
    
    /**
     * Returns an integer representing the tracking mode of the contour (whether
     * children moving along the contour rotate or not).
     */

    int getTrackingMode() const { return (int) _trackingMode; }
    
    /**
     * Returns an int representing whether the contour is visible or not.
     */

    int getVisible() const { return (int) _visible; }

    /**
     * Returns a float representing the thickness value of the contour.
     */

    float getThickness() const { return _thickness; }

    /**
     * Returns an integer representing the contour type according to the
     * contourType enum.
     */

    float getLineType() const { return (int) _lineType; }

    /**
     * Returns a Vector4 representing the RGBA value of the contour.
     */

    osg::Vec4 getColor() const { return _color;  }
    


     /**
      * Set whether the contour is influenced by lighting
     */
    void setLighting(int i);

    /**
     * Returns a boolean indicating whether the contour is affected by
     * lighting.
     */

    int getLighting() const { return (int)_lightingEnabled; }

	/**
	 * Draws the contour according to the parameters set, i.e. contour type,
	 * color, number of vertices.
	 */

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
    bool _lightingEnabled;
    
    ContourTypeEnum _lineType;
    float _thickness;

    osg::ref_ptr<osg::PositionAttitudeTransform> mainTransform;
    osg::ref_ptr<osg::Geode> vArrayGeode;
    //osg::ref_ptr<osg::Geometry> vArrayGeometry;

};

} // end of namespace spin

#endif
