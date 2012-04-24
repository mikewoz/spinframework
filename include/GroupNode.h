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


#ifndef __GroupNode_H
#define __GroupNode_H

#include "spinUtil.h"
#include "ReferencedNode.h"

#include <osg/Group>
#include <osg/PositionAttitudeTransform>
#include <osg/MatrixTransform>
#include <osg/Timer>
#include <osg/ClipNode>
#include <osgManipulator/Dragger>
#include <vector>

namespace spin
{

class UserNode;
class SceneManager;

/**
 * \brief A basic node to manage translation/orientation/scale of a subgraph.
 *        Allows for grouping of nodes.
 *
 * Most nodes in the scene do not have a method to offset their position in the
 * scene. This node provides a mechanism to group and offset nodes.
 */
class GroupNode : public ReferencedNode
{

public:

    /**
     * The constructor takes a string id that will be converted to a t_symbol,
     * which contains a pointer to this object. Thus functions will be able to
     * acquire the pointer using gensym(), which uses a hash table lookup.
     * @param initID will be converted into a t_symbol
     */

    GroupNode(SceneManager *sceneManager, char *initID);
    virtual ~GroupNode();

    enum InteractionMode
    {
        STATIC,     /*!< Mouse clicks and PointerNode have no effect on this
                    node. */
        PASSTHRU,   /*!< This passes the interaction event to the parent node,
                    which is useful when we need a node with geometry to be
                    selected, but want to move the parent group instead. NOTE:
                    ONLY IMPLEMENTED FOR PointerNode; NOT DONE YET FOR MOUSE! */
        SELECT,     /*!< This node responds to selection events (eg, can display
                    manipulator handles when selected). */
        DRAG,       /*!< This node responds to grag events, either invoked by
                    mouse drags in ViewerManipulator or PointerNode. */
        THROW,      /*!< The same as DRAG mode, but a direction vector is
                    accumulated over the last few drags and a velocity is
                    assigned to the node when released. */
        DRAW        /*!< Provides access to the x,y,z intersection points as the
                    user draws over the surface of a node */
    };
    enum globalsReportMode { NONE, GLOBAL_6DOF, GLOBAL_ALL };
    enum velocityMode { TRANSLATE, MOVE };
    
    enum OrientationMode
    {
        NORMAL,
        POINT_TO_TARGET,
        POINT_TO_TARGET_CENTROID,
        POINT_TO_ORIGIN
    };
    
    virtual void callbackUpdate();


    /**
     * IMPORTANT:
     * subclasses of ReferencedNode are allowed to contain complicated subgraphs
     * and can also change their attachmentNode so that children are attached
     * anywhere in that subgraph. If that is the case, the updateNodePath()
     * function MUST be overridden, and extra nodes must be manually pushed onto
     * currentNodePath.
     */
    virtual void updateNodePath(bool updateChildren = true);

    void mouseEvent (int event, int keyMask, int buttonMask, float x, float y);

    void event (int event, const char* userString, float eData1, float eData2,
    		float x, float y, float z);
	void setLock(const char* userString, int lock);


    virtual void debug();
    
    void setReportMode(globalsReportMode mode);

    void setInteractionMode(InteractionMode mode);
    
    /**
     * Set a clipping rectangle for the model so that geometry outside of the
     * region (+-x, +-y, +-z) will not be shown (or used in interactive events)
     */
    void setClipping(float x, float y, float z);
    
    /**
     * The local translation offset for this node with respect to it's parent
     */
    virtual void setTranslation (float x, float y, float z);

    /**
     * Set the OrientationMode of the node, which will be applied after every
     * transformation.
     */
    void setOrientationMode(OrientationMode m);
    int getOrientationMode() const { return (int)orientationMode_; };

    /**
     * The local orientation offset for this node with respect to it's parent
     */
    virtual void setOrientation (float pitch, float roll, float yaw);

    /**
     * Set the orientation offset as a quaternion
     */
    virtual void setOrientationQuat (float x, float y, float z, float w);

    void applyOrientationMode();

    /**
     * A grouped scale operation
     */
    virtual void setScale (float x, float y, float z);

    
    /**
     * A translational velocity (m/s). This is computed in the callbackUpdate()
     * function.
     */
    virtual void setVelocity (float dx, float dy, float dz);


    /**
     * Applying velocity to an object can either result in translational motion,
     * where velocityMode is TRANSLATE (0). This is the default and applies and
     * motion is relative to the local coordinate system of the node. When
     * velocityMode is MOVE (1), then motion is relative to the current
     * orientation of the node, analogous to the move() command.
     */
    virtual void setVelocityMode (velocityMode mode);


    /**
     * A rotational velocity (deg/sec), computed in callbackUpdate().
     */
    virtual void setSpin (float dp, float dr, float dy);

    /**
     * A Damping value (negative acceleration) that gets applied to velocity
     * and spin over time. Units are in -m/sec2 or -deg/sec2, meaning that:
     * - zero damping will not change velocity/spin values
     * - any positive value will decrease the speeds
     */
    virtual void setDamping (float d);
    
    /**
     * The translate command increments the node's current translation values
     * (ie, it's position in the scene with respect to it's parent)
     */
    virtual void translate (float x, float y, float z);
    
    /**
     * The move command adds a relative translation with respect to the
     * node's current orientation. That is, the node will translate along it's
     * direction vector by the supplied number of units.
     */
    virtual void move (float x, float y, float z);

    /**
     * The rotate command adds a relative rotation to the node's current
     * orientation.
     */
    virtual void rotate (float pitch, float roll, float yaw);


    virtual void setManipulator(const char *manipulatorType);
    const char* getManipulator() const { return manipulatorType_.c_str(); }

    virtual void setManipulatorMatrix
        (float a00, float a01, float a02, float a03,
         float a10, float a11, float a12, float a13,
         float a20, float a21, float a22, float a23,
         float a30, float a31, float a32, float a33);

    
    int getReportMode() const { return (int) _reportMode; };
    int getInteractionMode() const { return (int) _interactionMode; };
    osg::Vec3 getClipping() const { return _clipping; };
    osg::Vec3 getTranslation() const { return mainTransform->getMatrix().getTrans(); };
    osg::Vec3 getOrientation() const { return _orientation; };
    osg::Quat getOrientationQuat() const { return mainTransform->getMatrix().getRotate(); };
    osg::Vec3 getScale() const { return mainTransform->getMatrix().getScale(); };
    osg::Vec3 getVelocity() const { return _velocity; };
    int getVelocityMode() const { return (int) _velocityMode; };
    float getDamping() const { return _damping; };
    //osg::Vec3 getOrientation() { return Vec3inDegrees((mainTransform->getAttitude()).asVec3()); };

    osg::Matrix getGlobalMatrix();
    osg::Vec3 getCenter() const;

    
    /**
    * The dumpGlobals method results in a broadcast of this node's translation
    * and orientation. It is called by callbackUpdate() every frame, however the
    * 'forced' flag will be set to false, so it will only send a message if the
    * node's matrix has changed. If the 'forced' flag is set to true, it will
    * definitely result in a message broadcast. This should only be used when
    * necessary (eg, when a stateDump is requested).
    *
    * Note: the return value is only to fool wx so that it doesn't consider this
    * as an editable property.
    */
    bool dumpGlobals(bool forced);

    
    /**
     * For each subclass of ReferencedNode, we override the getState() method to
     * fill the vector with the correct set of methods for this particular node
     */
    virtual std::vector<lo_message> getState() const;

    /**
     * We override stateDump so that we can additionally force a dumpGlobals()
     * call whenever a dump is requested
     */
    virtual void stateDump();



    osg::MatrixTransform *getManipulatorTransform() { return manipulatorTransform.get(); }
    osg::MatrixTransform *getTransform() { return mainTransform.get(); }

protected:

    void updateDraggerMatrix();
    void updateMatrix();
    void drawManipulator();

    osg::ref_ptr<UserNode> owner;

    //osg::ref_ptr<osg::PositionAttitudeTransform> mainTransform;
    osg::ref_ptr<osg::MatrixTransform> mainTransform;
    osg::ref_ptr<osg::MatrixTransform> manipulatorTransform;
    
    osg::ref_ptr<osgManipulator::Dragger> dragger;
    
    InteractionMode _interactionMode;
    std::vector<osg::Vec4> _trajectory;
    int _drawMod;
    
    osg::ref_ptr<osg::ClipNode> clipNode;
    osg::Vec3 _clipping;
    
    globalsReportMode _reportMode;
    osg::Matrix _globalMatrix;
    osg::Vec3 _globalScale;
    float _globalRadius;

    enum OrientationMode orientationMode_;
    t_symbol* orientationTarget_;
    osg::Vec3 _orientation; // store the orientation as it comes in (in degrees)
    osg::Vec3 translation_;
    osg::Vec3 scale_;
    osg::Quat quat_;

    osg::Vec3 _velocity;
    velocityMode _velocityMode;
    osg::Vec3 _spin;
    float _damping;
    
    std::string manipulatorType_;
    bool manipulatorUpdateFlag_;
    bool manipulatorShadowCopy_;
    
private:

    
    osg::Timer_t lastTick;

};

    
class DraggerCallback : public osgManipulator::DraggerTransformCallback
{

public:
    DraggerCallback(GroupNode* g);
    bool receive(const osgManipulator::MotionCommand& command);
private:
    GroupNode *groupNode;
};
    
} // end of namespace spin

#endif
