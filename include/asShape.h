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
//  You should have received a copy of the Lesser GNU General Public License
//  along with SPIN Framework. If not, see <http://www.gnu.org/licenses/>.
// -----------------------------------------------------------------------------

#ifndef __ASSHAPE_H
#define __ASSHAPE_H

#include "asGlobals.h"
#include "asReferenced.h"

#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osg/Texture2D>
#include <osgUtil/Optimizer>


/**
 * \brief Node that represents a simple 3D geometry (spheres, boxes, etc).
 *
 * This allows for the creation of simple shapes (osg::Geodes) and provides a
 * mechanism for applying textures on the shape.
 */
class asShape : public asReferenced
{

public:

	asShape(asSceneManager *sceneManager, char *initID);
	virtual ~asShape();

	virtual void updateNodePath();

	void setTranslation (float x, float y, float z);
	void setOrientation (float pitch, float roll, float yaw);
	void setScale (float x, float y, float z);

    enum shapeType { NONE, SPHERE, BOX, CYLINDER, CAPSULE, CONE, PLANE };

	void setShape			(shapeType s);
	//void setShape			(char* newShape);

	void setColor			(float red, float green, float blue, float alpha);
	void setTexture			(int newTexture);
	void setTextureFromFile	(char* newTexture);
	void setRenderBin		(int i);

	osg::Vec3 getTranslation() { return shapeTransform->getPosition(); };
	osg::Vec3 getOrientation() { return _orientation; };
	osg::Vec3 getScale() { return shapeTransform->getScale(); };

	//char* getShape() { return (char*)shape.c_str(); }
	int getShape() { return (int)shape; }
	osg::Vec4 getColor() { return _color; };
	int getTexture() { return textureID; }
	int getRenderBin() { return renderBin; }

	/**
	 * For each subclass of asReferenced, we override the getState() method to
	 * fill the vector with the correct set of methods for this particular node
	 */
	virtual std::vector<lo_message> getState();

	/**
	 * We must include a stateDump() method that simply invokes the base class
	 * method. Simple C++ inheritance is not enough, because osg::Introspection
	 * won't see it.
	 */
	virtual void stateDump() { asReferenced::stateDump(); };




	// asShape supports simple graphical primitives (ie, spheres, boxes,
	// cylinders, cones, etc). The shape is attached to a shapeGeode, which
	// itself gets attached to the shapeTransform, allowing the shape to be offset
	// from it's parent.

	shapeType shape;
	//std::string shape;
	//std::string shapeDescr;

	osg::Vec4 _color;

	osg::Vec3 _orientation; // store the orientation as it comes in (in degrees)


	// We can have a texture on the shape, either from the database (given an id),
	// or loaded from a local file. Note that both methods cannot be valid at
	// the same time. ie, either textureID is 0, or textureFile is ""
	int textureID;
	std::string textureName;
	std::string texturePath;


	int renderBin;

	//osg::Vec3 _orientation;

	osg::ref_ptr<osg::Image> texturePointer; // store texturePointer so we don't waste time in the callback


	osg::ref_ptr<osg::PositionAttitudeTransform> shapeTransform;
	osg::ref_ptr<osg::Geode> shapeGeode;
	//osg::ref_ptr<osg::StateSet> shapeStateSet;

	osgUtil::Optimizer optimizer;


private:

	void drawShape();
	void drawTexture();

};


#endif
