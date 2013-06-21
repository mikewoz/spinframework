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

#ifndef _TextNode_H
#define _TextNode_H

#include "groupnode.h"

#include <osgText/Text>
#include <osgText/Text3D>
#include <osgText/String>
#include <osgText/TextBase>
/*
namespace osgText {
class Text;
class String;
}
*/

namespace osg {
class Geode;
}

namespace spin
{

class SceneManager;


// I'm including these two functions from OSG 3.1.1 because the wrap on the last
// line of text doesn't center properly in previous versions. When OSG packaging
// moves to 3.1+, we can remove this stuff
class spinTextNode : public osgText::Text
{
protected:
    osgText::String::iterator computeLastCharacterOnLine(osg::Vec2& cursor, osgText::String::iterator first, osgText::String::iterator last);
    void computeGlyphRepresentation();
};


/**
 * \brief Provides 3D text rendered in the scene
 */
class TextNode : public GroupNode
{

public:

    TextNode(SceneManager *sceneManager, const char* initID);
    virtual ~TextNode();

    virtual void callbackUpdate(osg::NodeVisitor* nv);

    enum DrawMode
    {
        GLYPH, /*!< characters are rendered to glyph textures */
        TEXT3D /*!< render 3D text with depth */
    };

    /**
     * The billboardType specifies how the text is oriented with respect to the
     * current camera position.
     */
    enum billboardType
    {
        RELATIVE,  /*!< No billboarding. */
        POINT_EYE, /*!< Set to rotate billboard around a camera's perspective */
        STAY_UP	   /*!< Billboard rotates only on z axis. */
    };

    /**
     * The decorationType specifies the type of dropshadow/outline used, which
     * can help the visibility of text on noisy backgrounds of similar color.
     */
    enum decorationType
    {
        // these are an exact copy of osgText::Text::BackdropType
        DROP_SHADOW_BOTTOM_RIGHT = 0,	/*!< Shadows down and to the right. */
        DROP_SHADOW_CENTER_RIGHT,		/*!< Shadows to the right. */
        DROP_SHADOW_TOP_RIGHT,			/*!< Shadows up and to the right. */
        DROP_SHADOW_BOTTOM_CENTER,		/*!< Shadows below. */
        DROP_SHADOW_TOP_CENTER,			/*!< Shadows upwards. */
        DROP_SHADOW_BOTTOM_LEFT,		/*!< Shadows down and to the left. */
        DROP_SHADOW_CENTER_LEFT,		/*!< Shadows to the left. */
        DROP_SHADOW_TOP_LEFT,			/*!< Shadows up and to the left. */
        OUTLINE,						/*!< Creates an outline of the text. */
        NONE							/*!< No shadowing or outline. */
    };

    /**
     * The backgroundType specifies the type of rectangle to draw around the
     * text (filled or wireframe). Use setMargin along with this to adjust the
     * appearance of a text box.
     */
    enum backgroundType {
    	NO_BACKGROUND,	/*!< The text box will have no background. */
    	FILLED, 		/*!< Creates a text box with a filled background. */
    	WIREFRAME, 		/*!< Creates a wireframe text box. */
    	ALL 			/*!< Creates a filled background with visible
						wireframe. */
    };

    virtual void setContext (const char *newvalue);

    /**
     * Choose whether the text is drawn in 2D glyphs, or as 3D geometry, with
	 * respect to the DrawMode enum.
     */
    void setDrawMode (DrawMode mode);

	/**
	 * Returns whether the text is drawn in 2d glyphs or as 3d geometry, with
	 * respect to the DrawMode enum.
	 */
	
    int  getDrawMode() const { return (int)drawMode_; }

    /**
     * Accepts user-entered string for the node's text.
     */
    void setText (const char* s);

	/**
	 * Returns the currently set text string.
	 */
    const char *getText() const   { return text_.c_str(); }

	/**
	 * Returns the currently set text string.
	 */
    std::string getTextString() const  { return text_; }

    /**
     * Deprecated method (here for backwards compatibility).
     */
    void setTextValue (const char* s) { setText(s); }

    /**
     * Set the number of times the text should repeat
     */
    void setRepetitions (int repetitions);

	/**
	 * Returns the current value for the number of text repititions.
	 */
    int  getRepetitions() const { return repetitions_; }
    

    /**
     * Sets the font for the text associated with this node.
     */
    void setFont (const char* s);

	/**
	 * Returns the currently set font associated to this text node.
	 */
    const char *getFont() const { return font_.c_str(); }

    /**
     * Set the font resolution. eg, 128 will produce 128x128 textures
     */
    void setFontResolution (int resolution);

	/**
	 * Returns the currently set font resolution.
	 */
    int  getFontResolution() const { return resolution_; }

    /**
     * Sets the size of text characters (in local coordinate system units)
     */
    void  setCharacterSize (float s);

	/**
	 * Returns the currently-set size of text characters (in local coordinate
	 * system units).
	 */
    float getCharacterSize() const { return characterSize_; }

    /**
     * Set the thickness of 3D text
     */
    void setThickness (float thickness);

	/**
	 * Returns the currently-set value of thickness for 3d text.
	 */
    float getThickness() const { return thickness_; }
    
    /**
     * Sets the maximum size of the text box. Values of 0 in either dimension
     * means no maximum, so that the box will stretch to fit the text
     */
    void setBoxSize (float width, float height);

    /**
     * Sets the line spacing, as a percentage of the character height. The
     * default is 0 
     */
    void setLineSpacing     (float spacing);

	/**
	 * Returns the currently-set line spacing as a percentage of character 
	 * height.
	 */
    float getLineSpacing() const { return lineSpacing_; }

    /**
     * Sets the maximum size of the text box.
     */
    void setAlignment       (int alignment);

    /**
     * Sets the color for the text associated to this node in RGBA values.
     */
    void setColor (float red, float green, float blue, float alpha);

	/**
	 * Returns the currently-set text color in RGBA values.
	 */
	
    osg::Vec4     getColor() const        { return color_; };
    
    /**
     * Sets the background color for this node.
     */
    void setBgColor (float red, float green, float blue, float alpha);
    osg::Vec4  getBgColor() const { return bgColor_; }
    
    /**
     * Sets the margins for the text associated to this node.
     */
    void  setMargin (float margin);

	/**
	 * Returns the currently-set margins for the text in the node.
	 */
    float getMargin() const { return margin_; }

    /**
     * Sets the type of billboarding asigned to this node (drawn from the enum
     * billboardType).
     */
    void setBillboard (billboardType t);
    int  getBillboard()  const { return (int)billboard_; }

    /**
     * Sets the shadowing or outline type for this text node (drawn from the
     * decorationType enum).
     */
    void setDecoration (decorationType t);
    int getDecoration() const { return (int)decoration_; }
    
    /**
     * Sets a background type for the text box (drawn from the backgroundType
     * enum).
     */
    void setBackground (backgroundType t);
    int  getBackround() const { return (int)background_; }
 
    /**
     * Specify whether both sides or only one side of the text is rendered. ie,
     * whether the backface is culled or not.
     */
    void setSingleSided (int singleSided);
    int  getSingleSided() const { return (int)singleSided_; }

    /**
     * Sets whether lighting has an effect on the text or not. If not, it is
	 * self-illuminated.
     */
    void setLighting (int lighting);

	/**
	 * Returns the currently-set lighting value.
	 */
    int  getLighting() const { return (int)lighting_; }
    
    /**
     * For each subclass of ReferencedNode, we override the getState() method to
     * fill the vector with the correct set of methods for this particular node
     */
    virtual std::vector<lo_message> getState() const;


private:

    bool updateFlag_;
    bool redrawFlag_;

    DrawMode drawMode_;
    std::string font_;
    int resolution_;
    float characterSize_, lineSpacing_, thickness_;
    osg::Vec4 color_, bgColor_;
    float margin_;
    osg::Vec2 boxSize_;
    
    int repetitions_;
    
    osgText::TextBase::AlignmentType alignment_;

    billboardType billboard_;
    decorationType decoration_;
    backgroundType background_;
    bool singleSided_;
    bool lighting_;
    

    std::string text_; // we store this redundantly

    osg::ref_ptr<osg::Geode> textGeode_;
    //osg::ref_ptr<spinTextNode> textLabel_;
    osg::ref_ptr<osgText::TextBase> textLabel_;
    
    void drawText();
    void updateText();

};


} // end of namespace spin

#endif // include guard
