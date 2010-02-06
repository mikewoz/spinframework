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

#ifndef __ShapeNode_H
#define __ShapeNode_H

#include "GroupNode.h"

#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osg/Texture2D>
#include <osgUtil/Optimizer>


/**
 * \brief Node that represents a simple 3D geometry (spheres, boxes, etc).
 *
 * This allows for the creation of simple graphical primitives (osg::Geodes such
 * as spheres, boxes, cylinders, cones, etc.) and provides some higher level
 * functionality, such as applying color and/or textures on the shape, setting
 * billboards, and organizing rendering order.
 * 
 * The shape is attached to "shapeGeode", which in turn, is attached to the
 * "mainTransform" inherited from GroupNode, allowing the shape to be offset
 * from it's parent.
 */
class ShapeNode : public GroupNode
{

public:

	ShapeNode(SceneManager *sceneManager, char *initID);
	virtual ~ShapeNode();

    /**
     * We provide several possible shapes
     */
    enum shapeType { NONE, SPHERE, BOX, CYLINDER, CAPSULE, CONE, PLANE };
    enum billboardType { RELATIVE, POINT_EYE, STAY_UP };
    
    virtual void setContext (const char *newvalue);

	void setShape			(shapeType t);
	void setBillboard		(billboardType t);

	void setColor			(float red, float green, float blue, float alpha);
	void setTextureFromFile	(const char* filename);
	void setRenderBin		(int i);
	void setLighting		(int i);


	int getShape() { return (int)shape; }
	int getBillboard() { return (int)billboard; }
	osg::Vec4 getColor() { return _color; };
	int getRenderBin() { return renderBin; }
	int getLighting() { return (int)lightingEnabled; }

	
	//void addSharedVideoTexture(osg::Node *n, std::string shID);
	//void addVideoTexture(osg::Node *n, std::string texturePath);
	void addImageTexture(osg::Node *n, std::string texturePath);
	
	/**
	 * For each subclass of ReferencedNode, we override the getState() method to
	 * fill the vector with the correct set of methods for this particular node
	 */
	virtual std::vector<lo_message> getState();



	shapeType shape;
	
	billboardType billboard;

	osg::Vec4 _color;

	// We can have a texture on the shape, loaded from a local file
	std::string texturePath;

	int renderBin;
	bool lightingEnabled;

	//osg::ref_ptr<osg::Image> textureImage; // store textureImage so we don't waste time in the callback

	osg::ref_ptr<osg::Geode> shapeGeode;

	osgUtil::Optimizer optimizer;


protected:

	virtual void drawShape();
	virtual void drawTexture();

};


#endif
