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

#include "GroupNode.h"


#include <osgText/Text>
#include <osg/PositionAttitudeTransform>



/**
 * \brief Provides 3D text rendered in the scene
 *
 */
class TextNode : public GroupNode
{

public:

	TextNode(SceneManager *sceneManager, char *initID);
	virtual ~TextNode();

    /**
     * We provide several possible shapes
     */
    enum billboardType { RELATIVE, POINT_EYE, STAY_UP };
    
	void setText			(const char* s);
	void setFont			(const char* s);
	void setBillboard		(billboardType t);
	void setColor			(float red, float green, float blue, float alpha);


	const char *getText() { return textLabel->getText().createUTF8EncodedString().c_str(); }
	const char *getFont() { return _font.c_str(); }
	int getBillboard() { return (int)_billboard; }
	osg::Vec4 getColor() { return _color; };


	/**
	 * For each subclass of ReferencedNode, we override the getState() method to
	 * fill the vector with the correct set of methods for this particular node
	 */
	virtual std::vector<lo_message> getState();


private:

	std::string _font;
	billboardType _billboard;
	osg::Vec4 _color;

	osg::ref_ptr<osg::Geode> textGeode;
	osg::ref_ptr<osgText::Text> textLabel;
	
	void drawText();

};


#endif
