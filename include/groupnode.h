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

#include "spinutil.h"
#include "referencednode.h"

#include <osg/Group>
#include <osg/PositionAttitudeTransform>
#include <osg/MatrixTransform>
#include <osgAnimation/EaseMotion>
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

    GroupNode(SceneManager *sceneManager, const char* initID);
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
	
    enum globalsReportMode 
	{ 
		NONE, 
		GLOBAL_6DOF, 
		GLOBAL_ALL 
	};
	
    enum velocityMode 
	{ 	
		TRANSLATE, 
		MOVE 
	};
    enum ComputationMode 
	{ 	
		SERVER_SIDE, 
		CLIENT_SIDE 
	};
	
    enum OrientationMode
    {
        NORMAL,
        POINT_TO_TARGET,
        POINT_TO_TARGET_CENTROID,
        POINT_TO_ORIGIN
    };
    
    virtual void callbackUpdate(osg::NodeVisitor* nv);

    /**
     * The update rate tells the server how fast to send both velocity updates
     * and reporting. Generally, we want to throttle network messages on the
     * server-side so that messages don't flood the network when there is a lot
     * of activity.
     */
    virtual void setUpdateRate(float seconds);

	/**
	 * Returns the currently-set update rate.
	 */
	
    float getUpdateRate() const { return maxUpdateDelta_; }

    /**
     * IMPORTANT:
     * subclasses of ReferencedNode are allowed to contain complicated subgraphs
     * and can also change their attachmentNode so that children are attached
     * anywhere in that subgraph. If that is the case, the updateNodePath()
     * function MUST be overridden, and extra nodes must be manually pushed onto
     * currentNodePath_.
     */
    virtual void updateNodePath(bool updateChildren = true);

    void mouseEvent (int event, int keyMask, int buttonMask, float x, float y);

    void event (int event, const char* userString, float eData1, float eData2,
    		float x, float y, float z);
	void setLock(const char* userString, int lock);

    /**
     * Print debug information about the node to standard out (when running in
     * console mode). It may be possible to redirect this to a text box for GUI
     * logs.
     */
    virtual void debug();
    
    /**
     * setStateSetFromFile guesses the type of stateset from the filename
     * extension, creates a new stateset of that type and assigns it to this
     * node
     */
    void setStateSetFromFile(const char* filename);
    
    /**
     * Assign an existing stateset to this node
     */
    void setStateSet(const char* s);
    t_symbol* getStateSetSymbol() const { return stateset_; }
    
    /**
     * This method actually applies the stateset to the subgraph, replacing any
     * existing stateset with this one. The setStateSet and setStateSetFromFile
     * methods just set the stateset_ symbol, while updateStateSet does the
     * actual work.
     *
     * Override this method in subclasses in order to change how stateset should
     * be applied. For example, to which node in the subgraph it should be
     * attached, or whether it should be merged with the existing stateset
     * (rather than merged).
     *
     * By default it is applied to the mainTransform_.
     */
    virtual void updateStateSet();

	/**
	 * Sets the report mode with reference to the globalsReportMode enum.
	 */
    
    void setReportMode(globalsReportMode mode);

	/**
	 * Sets the interaction mode with reference to the InteractionMode enum.
	 */

    void setInteractionMode(InteractionMode mode);

	/**
	 * Sets the Computation mode as either server or client side with respect to
	 * the ComputationMode enum.
	 */
    
    void setComputationMode(ComputationMode mode);

	/**
	 * Returns the currently-set computation mode with respect to the
	 * ComputationMode enum.
	 */
	
    int getComputationMode() const { return (int)computationMode_; };

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

	/**
	 * Returns the currently-set Orientation Mode, with respect to the
	 * OrientationMode enum.
	 */
	
    int getOrientationMode() const { return (int)orientationMode_; };

    void setOrientationTarget(const char* target);
    char* getOrientationTarget() const { return orientationTarget_->s_name; };

    /**
     * The local orientation offset for this node with respect to its parent
     */
    virtual void setOrientation (float pitch, float roll, float yaw);

    /**
     * Set the orientation offset as a quaternion
     */
    virtual void setOrientationQuat (float x, float y, float z, float w);

    /**
     * A grouped scale operation.
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
     * The rotate command adds to the (absolute) orientation of the node
     */
    virtual void rotate (float dPitch, float dRoll, float dYaw);

    /**
     * The addRotation command adds a (relative) rotation to the node's current
     * orientation.
     */
    virtual void addRotation (float dPitch, float dRoll, float dYaw);

    /**
     * Instead of instantaneous setTranslation, this method uses an ease motion
     * to animate the node to the target position.
     */
    virtual void translateTo (float x, float y, float z, float time, const char *motion="Linear");

    virtual void setManipulator(const char *manipulatorType);
    const char* getManipulator() const { return manipulatorType_.c_str(); }

    virtual void setManipulatorMatrix
        (float a00, float a01, float a02, float a03,
         float a10, float a11, float a12, float a13,
         float a20, float a21, float a22, float a23,
         float a30, float a31, float a32, float a33);
         
    void setBroadcastLock(bool lock) { broadcastLock_ = lock; }

	/**
	 * Returns the currently-set Report Mode with reference to the
	 * globalsReportMode enum.
	 */
	
    int getReportMode() const { return (int) reportMode_; };

	/**
	 * Returns the currently-set Interaction Mode with reference to the
	 * InteractionMode enum.
	 */
	
    int getInteractionMode() const { return (int) interactionMode_; };
    osg::Vec3 getClipping() const { return clipping_; };

	/**
	 * Returns the currently-set local orientation offset for this node with
	 * respect to its parent.
	 */
	
    osg::Vec3 getOrientation() const { return orientation_; };
    
    /*
    osg::Vec3 getTranslation() const { return mainTransform_->getMatrix().getTrans(); };
    osg::Quat getOrientationQuat() const { return mainTransform_->getMatrix().getRotate(); };
    osg::Vec3 getScale() const { return mainTransform_->getMatrix().getScale(); };
    */

	/**
	 * Returns the currently-set local translation offset for this node with
	 * respect to its parent.
	 */
	
    virtual osg::Vec3 getTranslation() const { return translation_; };

	/**
	 * Returns the currently-set 
	 */
	
    virtual osg::Quat getOrientationQuat() const { return quat_; };
    virtual osg::Vec3 getScale() const { return scale_; };     
    
    osg::Vec3 getVelocity() const { return velocity_; };
    int getVelocityMode() const { return (int) velocityMode_; };
    float getDamping() const { return damping_; };
    //osg::Vec3 getOrientation() { return Vec3inDegrees((mainTransform_->getAttitude()).asVec3()); };

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
    virtual bool dumpGlobals(bool forced=false);

    
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



    osg::MatrixTransform *getTransform() { return mainTransform_.get(); }

    void updateDraggerMatrix();

    void removeOrientationTargetter( GroupNode* gn );
    void addOrientationTargetter( GroupNode* gn );
    void applyOrientationModeToTargetters();

protected:

    void updateQuat();
    void updateMatrix();
    void drawManipulator();
    
    void applyOrientationMode();


    osg::ref_ptr<UserNode> owner_;
    
    t_symbol *stateset_;

    osg::ref_ptr<osg::MatrixTransform> mainTransform_;
    
    osg::ref_ptr<osgManipulator::Dragger> dragger_;
    
    osg::ref_ptr<osgAnimation::Motion> motion_;
    osg::Vec3 motionStart_, motionEnd_;
    pthread_mutex_t motionMutex_;

    InteractionMode interactionMode_;
    std::vector<osg::Vec4> trajectory_;
    int drawMod_;
    
    osg::ref_ptr<osg::ClipNode> clipNode_;
    osg::Vec3 clipping_;
    
    globalsReportMode reportMode_;
    osg::Matrix globalMatrix_;
    osg::Vec3 _globalScale;
    float globalRadius_;

    ComputationMode computationMode_;
    float maxUpdateDelta_;

    enum OrientationMode orientationMode_;
    t_symbol* orientationTarget_;
    osg::Vec3 orientation_; // store the orientation as it comes in (in degrees)
    osg::Vec3 translation_;
    osg::Vec3 scale_;
    osg::Quat quat_;

    osg::Vec3 velocity_;
    velocityMode velocityMode_;
    osg::Vec3 spin_;
    float damping_;
    
    std::string manipulatorType_;
    bool manipulatorUpdateFlag_;
    bool manipulatorShadowCopy_;

    bool broadcastLock_;

    typedef std::vector<GroupNode*> GroupNodeList;
    GroupNodeList orientationTargetters_;
    
private:

    // make sure this is kept private because subclasses might have different
    // schedulers
    osg::Timer_t lastTick_;
    osg::Timer_t lastUpdate_;
    
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
