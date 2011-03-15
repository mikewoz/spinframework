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

namespace osgText {
class Text;
}

namespace osg {
class Geode;
}

class SceneManager;

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
     * The billboardType specifies how the text is oriented with respect to the
     * current camera position.
     */
    enum billboardType
    {
        RELATIVE,
        POINT_EYE,
        STAY_UP
    };

    /**
     * The decorationType specifies the type of dropshadow/outline used, which
     * can help the visibility of the text on noisy of similarly colored
     * backgrounds.
     */
    enum decorationType
    {
        // these are an exact copy of osgText::Text::BackdropType
        DROP_SHADOW_BOTTOM_RIGHT = 0,
        DROP_SHADOW_CENTER_RIGHT,
        DROP_SHADOW_TOP_RIGHT,
        DROP_SHADOW_BOTTOM_CENTER,
        DROP_SHADOW_TOP_CENTER,
        DROP_SHADOW_BOTTOM_LEFT,
        DROP_SHADOW_CENTER_LEFT,
        DROP_SHADOW_TOP_LEFT,
        OUTLINE,
        NONE
    };

    /**
     * The backgroundType specifies the type of rectangle to draw around the
     * text (filled or wireframe). Use setMargin along with this to adjust the
     * appearance of a text box.
     */
    enum backgroundType { NO_BACKGROUND, FILLED, WIREFRAME, ALL };


    virtual void setContext    (const char *newvalue);
    
    void setTextValue        (const char* s);
    void setFont            (const char* s);
    void setSize            (float s);
    void setColor            (float red, float green, float blue, float alpha);
    void setBgColor            (float red, float green, float blue, float alpha);
    void setMargin            (float margin);

    void setBillboard        (billboardType t);
    void setDecoration        (decorationType t);
    void setBackground        (backgroundType t);


    //const char *getTextValue() { return textLabel->getText().createUTF8EncodedString().c_str(); }
    const char    *getTextValue() const   { return _text.c_str(); }
    std::string     getTextString() const  { return _text; }
    const char    *getFont() const        { return _font.c_str(); }
    float         getSize() const        { return _size; }
    osg::Vec4     getColor() const        { return _color; };
    osg::Vec4     getBgColor() const        { return _bgColor; }
    float         getMargin() const        { return _margin; }

    int             getBillboard()  const { return (int)_billboard; }
    int             getDecoration() const { return (int)_decoration; }
    int             getBackround() const   { return (int)_background; }

    /**
     * For each subclass of ReferencedNode, we override the getState() method to
     * fill the vector with the correct set of methods for this particular node
     */
    virtual std::vector<lo_message> getState() const;


private:

    std::string _font;
    float _size;
    osg::Vec4 _color, _bgColor;
    float _margin;

    billboardType _billboard;
    decorationType _decoration;
    backgroundType _background;

    std::string _text; // we store this redundantly

    osg::ref_ptr<osg::Geode> textGeode;
    osg::ref_ptr<osgText::Text> textLabel;
    
    void drawText();

};


#endif
