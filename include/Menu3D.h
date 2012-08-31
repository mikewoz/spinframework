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

#include <osg/Switch>
#include "GroupNode.h"
#include "TextNode.h"

namespace spin
{

typedef std::vector< osg::observer_ptr<TextNode> > MenuVector;

/**
 * \brief Provides a positionable 3D menu composed of TextNodes
 */
class Menu3D : public GroupNode
{

public:

    Menu3D(SceneManager *sceneManager, const char* initID);
    virtual ~Menu3D();

    /**
     * This should likely be a private function. TO_BE_VERIFIED.
     */

    virtual void updateNodePath();

    /**
     * Toggle whether this menu node is enabled or disabled.
     */

    void setEnabled(int i);

    /**
     * Returns a boolean (int) indicating whether this menu node is enabled or
     * disabled.
     */

    int getEnabled() const { return enabled_; }
    
    /**
     * Add an item (TextNode) to the list (server-side only)
     */
    void addItem (const char *itemText);

    /**
     * Remove an item (textNode) from the list by its itemIndex.
     */
    void removeItem (int itemIndex);

    /**
     * Remove an item (textNode) from the list by its ID.
     */

    void removeItem (const char *itemID);

    /**
     * This should be a private function.
     */

    int doRemoveItem (osg::observer_ptr<TextNode> n);

    /**
     * Removes all text nodes from the menu.
     */

    void clearItems();

    /**
     * Redraws the menu after items are removed to eliminate the space left by
     * the recently removed item(s).
     */

    void redraw();
    
    /**
     * Highlight an item from the list by its itemIndex.
     */
    void setHighlighted(int itemIndex);

    /**
     * Highlight an item from the list by its itemID.
     */

    void setHighlighted(const char *itemID);

    /**
     * This should be a private function.
     */

    int doHighlight(osg::observer_ptr<TextNode> n);

    /**
     * Returns the id of the currently highlighted menu item.
     */

    const char *getHighlighted() const { if (highlighted_.valid())
    	return highlighted_->getID().c_str(); else return "NULL"; }



    /**
     * Set the color of the font in RGBA values when highlighted
     */

    void setHighlightColor(float r, float g, float b, float a);

    /**
     * Returns the color of the font in RGBA values when highlighted.
     */

    osg::Vec4 getHighlightColor() const { return highlightColor_; }

    /**
     * Highlights the item that follows the currently highlighted item.
     */

    void highlightNext();

    /**
     * Highlights the item previous to the currently highlighted item.
     */

    void highlightPrev();

    void select();
    
    /**
     * Each successive menu item will appear at an offset from the previous
     */
    void setItemOffset(float x, float y, float z);

    /**
     * Returns the Vector3 value used to offset successive menu entries.
     */

    osg::Vec3 getItemOffset() const { return itemOffset_; }
    
    /**
     * wrapped from TextNode: sets font type
     */
    void setFont            (const char* s);
    /**
     * wrapped from TextNode:
     * types come from billboardType enum: RELATIVE, POINT_EYE, STAY_UP
     */
    void setBillboard        (TextNode::billboardType t);
    /**
     * wrapped from TextNode: sets color in RGBA values
     */
    void setColor            (float red, float green, float blue, float alpha);

    /**
     * For each subclass of ReferencedNode, we override the getState() method to
     * fill the vector with the correct set of methods for this particular node
     */
    virtual std::vector<lo_message> getState() const;


private:


    osg::observer_ptr<TextNode> highlighted_;

    MenuVector items_;
    osg::Vec3 itemOffset_;
    
    int enabled_;
    std::string font_;
    TextNode::billboardType billboardType_;
    osg::Vec4 color_;
    osg::Vec4 highlightColor_;
    
    osg::ref_ptr<osg::Switch> switcher;
};

} // end of namespace spin

#endif
