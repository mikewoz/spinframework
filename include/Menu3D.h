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

#ifndef __Menu3D_H
#define __Menu3D_H

#include "GroupNode.h"
#include "TextNode.h"

typedef std::vector< osg::observer_ptr<TextNode> > MenuVector;


/**
 * \brief Provides a positionable 3D menu composed of TextNodes
 *
 */
class Menu3D : public GroupNode
{

public:

	Menu3D(SceneManager *sceneManager, char *initID);
	virtual ~Menu3D();

	/**
	 * Add an item (TextNode) to the list
	 */
	void addItem (const char *itemText);

	/**
	 * Remove an item from the list
	 */
	void removeItem (int itemIndex);
	void removeItem (const char *itemID);
	int doRemoveItem (osg::observer_ptr<TextNode> n);

	void clearItems();
	void redraw();
	
	/**
	 * Highlight an item from the list
	 */
	void setHighlighted(int itemIndex);
	void setHighlighted(const char *itemID);
	int doHighlight(osg::observer_ptr<TextNode> n);
	const char *getHighlighted() { if (highlighted_.valid()) return highlighted_->id->s_name; else return "NULL"; }



	/**
	 * Set the color of the font when highlighted
	 */
	void setHighlightColor(float r, float g, float b, float a);
	osg::Vec4 getHighlightColor() { return highlightColor_; }

	
	/**
	 * wrapped from TextNode:
	 */
	void setFont			(const char* s);
	/**
	 * wrapped from TextNode:
	 */
	void setBillboard		(TextNode::billboardType t);
	/**
	 * wrapped from TextNode:
	 */
	void setColor			(float red, float green, float blue, float alpha);

	/**
	 * For each subclass of ReferencedNode, we override the getState() method to
	 * fill the vector with the correct set of methods for this particular node
	 */
	virtual std::vector<lo_message> getState();


private:


	osg::observer_ptr<TextNode> highlighted_;

	MenuVector items_;

	std::string font_;
	TextNode::billboardType billboardType_;
	osg::Vec4 color_;
	osg::Vec4 highlightColor_;


};


#endif
