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

#ifndef __CameraNode_H
#define __CameraNode_H

#include <string>
#include <osg/Vec3>
#include <osg/PositionAttitudeTransform>

#include "ReferencedNode.h"



/**
 * \brief Defines a camera perspective located within the scene (NOT YET COMPLETE)
 *
 * This class is used to define camera properties, and is meant to be used with
 * a NodeTrackerManipulator in an osgViewer application
 */

class CameraNode : public ReferencedNode
{

public:

	CameraNode (SceneManager *sceneManager, char *initID);
	virtual ~CameraNode();

	/**
	 * IMPORTANT:
	 * subclasses of ReferencedNode are allowed to contain complicated subgraphs,
	 * and can also change their attachmentNode so that children are attached
	 * anywhere in that subgraph. If that is the case, the updateNodePath()
	 * function MUST be overridden, and extra nodes must be manually pushed onto
	 * currentNodePath.
	 */
	virtual void updateNodePath();

	void setTranslation	(float x, float y, float z);
	void setOrientation	(float pitch, float roll, float yaw);
	void setFrustum		(float left, float right, float bottom, float top, float zNear, float zFar);
	void setViewport	(float x, float y, float width, float height);
	void setClearColor	(float red, float green, float blue, float alpha);
	
	
	
	osg::Vec3 getTranslation() { return modelTransform->getPosition(); };
	osg::Vec3 getOrientation() { return _orientation; };

	

	/**
	 * For each subclass of ReferencedNode, we override the getState() method to
	 * fill the vector with the correct set of methods for this particular node
	 */
	virtual std::vector<lo_message> getState();


private:


	// We allow users to offset the camera and store this offset in the
	// cameraTransform node
	osg::ref_ptr<osg::PositionAttitudeTransform> cameraTransform;

	osg::Vec3 _orientation; // store the orientation as it comes in (in degrees)

	osg::Vec4 _viewport;
	osg::Vec4 _clearColor;
	osg::Vec4 _frustumXY;
	float _frustumNear;
	float _frustumFar;
	
};



#endif
