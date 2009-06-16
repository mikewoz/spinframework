// ===================================================================
// Audioscape library for PureData
// Copyright (c) 2007
//
// Collaborators:
//    Shared Reality Lab (SRE), McGill University Centre for Intelligent Machines (CIM)
//       www.cim.mcgill.ca/sre
//    La Société des Arts Technologiques (SAT)
//       www.sat.qc.ca
//
// Project Directors:
//    Science - Jeremy R. Cooperstock (SRE/CIM)
//    Arts - Zack Settel
//
// Conception:
//    Zack Settel
//
// Development Team:
//    Mike Wozniewski (SRE/CIM): Researcher, Head Developer
//    Zack Settel: Artist, Researcher, Audio/DSP programming
//    Jean-Michel Dumas (SAT): Assistant Researcher
//    Mitchel Benovoy (SRE/CIM): Video Texture Programming
//    Stéphane Pelletier (SRE/CIM): Video Texture Programming
//    Pierre-Olivier Charlebois (SRE/CIM): Former Developer
//
// Funding by / Souventionné par:
//    Natural Sciences and Engineering Research Council of Canada (NSERC)
//    Canada Council for the Arts
//    NSERC/Canada Council for the Arts - New Media Initiative
//
// ===================================================================
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
// ===================================================================


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

	void setColor			(float r, float g, float b, float a);
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


	void drawShape();
	void drawTexture();

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


};


#endif
