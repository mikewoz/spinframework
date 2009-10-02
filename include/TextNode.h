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

#ifndef __TextNode_H
#define __TextNode_H

#include "ReferencedNode.h"


#include <osgText/Text>
#include <osg/PositionAttitudeTransform>



/**
 * \brief Provides 3D text rendered in the scene
 *
 */
class TextNode : public ReferencedNode
{

public:

	TextNode(SceneManager *sceneManager, char *initID);
	virtual ~TextNode();

	/**
	 * IMPORTANT:
	 * subclasses of ReferencedNode are allowed to contain complicated subgraphs,
	 * and can also change their attachmentNode so that children are attached
	 * anywhere in that subgraph. If that is the case, the updateNodePath()
	 * function MUST be overridden, and extra nodes must be manually pushed onto
	 * currentNodePath.
	 */
	virtual void updateNodePath();


    /**
     * We provide several possible shapes
     */
    enum billboardType { RELATIVE, POINT_EYE, STAY_UP };
    
	void setText			(const char* s);
	void setFont			(const char* s);
	void setBillboard		(billboardType t);
	void setColor			(float red, float green, float blue, float alpha);

    /**
     * This is a local translational offset from the parent
     */
	void setTranslation (float x, float y, float z);

    /**
     * This is a local orientation offset from the parent
     */
	void setOrientation (float pitch, float roll, float yaw);

	 /**
     * Allows for scaling in each axis
     */
	void setScale (float x, float y, float z);


	const char *getText() { return _text.c_str(); }
	const char *getFont() { return _font.c_str(); }
	int getBillboard() { return (int)_billboard; }
	osg::Vec4 getColor() { return _color; };
    osg::Vec3 getTranslation() { return textTransform->getPosition(); };
	osg::Vec3 getOrientation() { return _orientation; };
	osg::Vec3 getScale() { return textTransform->getScale(); };


	/**
	 * For each subclass of ReferencedNode, we override the getState() method to
	 * fill the vector with the correct set of methods for this particular node
	 */
	virtual std::vector<lo_message> getState();



	std::string _text, _font;
	
	billboardType _billboard;

	osg::Vec4 _color;

	osg::Vec3 _orientation; // store the orientation as it comes in (in degrees)


	osg::ref_ptr<osg::PositionAttitudeTransform> textTransform;


private:
	
	osg::ref_ptr<osg::Geode> textGeode;
	
	void drawText();

};


#endif
