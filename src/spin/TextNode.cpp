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

#include <osgText/Text>
#include <osgText/TextBase>
#include <osg/Billboard>
#include <osg/Version>
#include <lo/lo_types.h>

#include "osgutil.h"
#include "textnode.h"
#include "scenemanager.h"
#include "spinapp.h"
#include "spinbasecontext.h"

extern pthread_mutex_t sceneMutex;

namespace spin
{

void spinTextNode::computeGlyphRepresentation()
{

#ifdef OSG_MIN_VERSION_REQUIRED
#if OSG_MIN_VERSION_REQUIRED(3,0,0)

    using namespace osgText;

    Font* activefont = getActiveFont();
    if (!activefont) return;
    
    _textureGlyphQuadMap.clear();
    _lineCount = 0;
    
    if (_text.empty()) 
    {
        _textBB.set(0,0,0,0,0,0);//no size text
        TextBase::computePositions(); //to reset the origin
        return;
    }
    
    //OpenThreads::ScopedLock<Font::FontMutex> lock(*(activefont->getSerializeFontCallsMutex()));

    // initialize bounding box, it will be expanded during glyph position calculation
    _textBB.init();

    osg::Vec2 startOfLine_coords(0.0f,0.0f);
    osg::Vec2 cursor(startOfLine_coords);
    osg::Vec2 local(0.0f,0.0f);
    
    unsigned int previous_charcode = 0;
    unsigned int linelength = 0;
    bool horizontal = _layout!=VERTICAL;
    bool kerning = true;
    
    unsigned int lineNumber = 0;

    float hr = _characterHeight;
    float wr = hr/getCharacterAspectRatio();

    for(String::iterator itr=_text.begin();
        itr!=_text.end();
        )
    {
        // record the start of the current line
            String::iterator startOfLine_itr = itr;

            // find the end of the current line.
            osg::Vec2 endOfLine_coords(cursor);
            String::iterator endOfLine_itr = computeLastCharacterOnLine(endOfLine_coords, itr,_text.end());
            
            linelength = endOfLine_itr - startOfLine_itr;

            // Set line position to correct alignment.
            switch(_layout)
            {
            case LEFT_TO_RIGHT:
            {
            switch(_alignment)
            {
              // nothing to be done for these
              //case LEFT_TOP:
              //case LEFT_CENTER:
              //case LEFT_BOTTOM:
              //case LEFT_BASE_LINE:
              //case LEFT_BOTTOM_BASE_LINE:
              //  break;
              case CENTER_TOP:
              case CENTER_CENTER:
              case CENTER_BOTTOM:
              case CENTER_BASE_LINE:
              case CENTER_BOTTOM_BASE_LINE:
                cursor.x() = (cursor.x() - endOfLine_coords.x()) * 0.5f;
                break;
              case RIGHT_TOP:
              case RIGHT_CENTER:
              case RIGHT_BOTTOM:
              case RIGHT_BASE_LINE:
              case RIGHT_BOTTOM_BASE_LINE:
                cursor.x() = cursor.x() - endOfLine_coords.x();
                break;
              default:
                break;
              }
            break;
            }
            case RIGHT_TO_LEFT:
            {
            switch(_alignment)
            {
              case LEFT_TOP:
              case LEFT_CENTER:
              case LEFT_BOTTOM:
              case LEFT_BASE_LINE:
              case LEFT_BOTTOM_BASE_LINE:
                cursor.x() = 2*cursor.x() - endOfLine_coords.x();
                break;
              case CENTER_TOP:
              case CENTER_CENTER:
              case CENTER_BOTTOM:
              case CENTER_BASE_LINE:
              case CENTER_BOTTOM_BASE_LINE:
                cursor.x() = cursor.x() + (cursor.x() - endOfLine_coords.x()) * 0.5f;
                break;
              // nothing to be done for these
              //case RIGHT_TOP:
              //case RIGHT_CENTER:
              //case RIGHT_BOTTOM:
              //case RIGHT_BASE_LINE:
              //case RIGHT_BOTTOM_BASE_LINE:
              //  break;
              default:
                break;
            }
            break;
            }
            case VERTICAL:
            {
            switch(_alignment)
            {
              // TODO: current behaviour top baselines lined up in both cases - need to implement
              //       top of characters alignment - Question is this necessary?
              // ... otherwise, nothing to be done for these 6 cases
              //case LEFT_TOP:
              //case CENTER_TOP:
              //case RIGHT_TOP:
              //  break;
              //case LEFT_BASE_LINE:
              //case CENTER_BASE_LINE:
              //case RIGHT_BASE_LINE:
              //  break;
              case LEFT_CENTER:
              case CENTER_CENTER:
              case RIGHT_CENTER:
                cursor.y() = cursor.y() + (cursor.y() - endOfLine_coords.y()) * 0.5f;
                break;
              case LEFT_BOTTOM_BASE_LINE:
              case CENTER_BOTTOM_BASE_LINE:
              case RIGHT_BOTTOM_BASE_LINE:
                cursor.y() = cursor.y() - (linelength * _characterHeight);
                break;
              case LEFT_BOTTOM:
              case CENTER_BOTTOM:
              case RIGHT_BOTTOM:
                cursor.y() = 2*cursor.y() - endOfLine_coords.y();
                break;
              default:
                break;
            }
            break;
          }
        }

        if (itr!=endOfLine_itr)
        {

            for(;itr!=endOfLine_itr;++itr)
            {
                unsigned int charcode = *itr;

                Glyph* glyph = activefont->getGlyph(_fontSize, charcode);
                if (glyph)
                {
                    float width = (float)(glyph->getWidth()) * wr;
                    float height = (float)(glyph->getHeight()) * hr;

                    if (_layout==RIGHT_TO_LEFT)
                    {
                        cursor.x() -= glyph->getHorizontalAdvance() * wr;
                    }

                    // adjust cursor position w.r.t any kerning.
                    if (kerning && previous_charcode)
                    {
                        switch(_layout)
                        {
                          case LEFT_TO_RIGHT:
                          {
                            osg::Vec2 delta(activefont->getKerning(previous_charcode,charcode,_kerningType));
                            cursor.x() += delta.x() * wr;
                            cursor.y() += delta.y() * hr;
                            break;
                          }
                          case RIGHT_TO_LEFT:
                          {
                            osg::Vec2 delta(activefont->getKerning(charcode,previous_charcode,_kerningType));
                            cursor.x() -= delta.x() * wr;
                            cursor.y() -= delta.y() * hr;
                            break;
                          }
                          case VERTICAL:
                            break; // no kerning when vertical.
                        }
                    }

                    local = cursor;
                    osg::Vec2 bearing(horizontal?glyph->getHorizontalBearing():glyph->getVerticalBearing());
                    local.x() += bearing.x() * wr;
                    local.y() += bearing.y() * hr;

                    GlyphQuads& glyphquad = _textureGlyphQuadMap[glyph->getTexture()];

                    glyphquad._glyphs.push_back(glyph);
                    glyphquad._lineNumbers.push_back(lineNumber);

                    // Adjust coordinates and texture coordinates to avoid
                    // clipping the edges of antialiased characters.
                    osg::Vec2 mintc = glyph->getMinTexCoord();
                    osg::Vec2 maxtc = glyph->getMaxTexCoord();
                    osg::Vec2 vDiff = maxtc - mintc;

                    float fHorizTCMargin = 1.0f / glyph->getTexture()->getTextureWidth();
                    float fVertTCMargin = 1.0f / glyph->getTexture()->getTextureHeight();
                    float fHorizQuadMargin = vDiff.x() == 0.0f ? 0.0f : width * fHorizTCMargin / vDiff.x();
                    float fVertQuadMargin = vDiff.y() == 0.0f ? 0.0f : height * fVertTCMargin / vDiff.y();

                    mintc.x() -= fHorizTCMargin;
                    mintc.y() -= fVertTCMargin;
                    maxtc.x() += fHorizTCMargin;
                    maxtc.y() += fVertTCMargin;

                    // set up the coords of the quad
                    osg::Vec2 upLeft = local+osg::Vec2(0.0f-fHorizQuadMargin,height+fVertQuadMargin);
                    osg::Vec2 lowLeft = local+osg::Vec2(0.0f-fHorizQuadMargin,0.0f-fVertQuadMargin);
                    osg::Vec2 lowRight = local+osg::Vec2(width+fHorizQuadMargin,0.0f-fVertQuadMargin);
                    osg::Vec2 upRight = local+osg::Vec2(width+fHorizQuadMargin,height+fVertQuadMargin);
                    glyphquad._coords.push_back(upLeft);
                    glyphquad._coords.push_back(lowLeft);
                    glyphquad._coords.push_back(lowRight);
                    glyphquad._coords.push_back(upRight);

                    // set up the tex coords of the quad
                    glyphquad._texcoords.push_back(osg::Vec2(mintc.x(),maxtc.y()));
                    glyphquad._texcoords.push_back(osg::Vec2(mintc.x(),mintc.y()));
                    glyphquad._texcoords.push_back(osg::Vec2(maxtc.x(),mintc.y()));
                    glyphquad._texcoords.push_back(osg::Vec2(maxtc.x(),maxtc.y()));

                    // move the cursor onto the next character.
                    // also expand bounding box
                    switch(_layout)
                    {
                      case LEFT_TO_RIGHT:
                          cursor.x() += glyph->getHorizontalAdvance() * wr;
                          _textBB.expandBy(osg::Vec3(lowLeft.x(), lowLeft.y(), 0.0f)); //lower left corner
                          _textBB.expandBy(osg::Vec3(upRight.x(), upRight.y(), 0.0f)); //upper right corner
                          break;
                      case VERTICAL:
                          cursor.y() -= glyph->getVerticalAdvance() * hr;
                          _textBB.expandBy(osg::Vec3(upLeft.x(),upLeft.y(),0.0f)); //upper left corner
                          _textBB.expandBy(osg::Vec3(lowRight.x(),lowRight.y(),0.0f)); //lower right corner
                          break;
                      case RIGHT_TO_LEFT:
                          _textBB.expandBy(osg::Vec3(lowRight.x(),lowRight.y(),0.0f)); //lower right corner
                          _textBB.expandBy(osg::Vec3(upLeft.x(),upLeft.y(),0.0f)); //upper left corner
                          break;
                    }
                    previous_charcode = charcode;

                }
            }

            // skip over spaces and return.
            while (itr != _text.end() && *itr==' ') ++itr;
            if (itr != _text.end() && *itr=='\n') ++itr;
        }
        else
        {
            ++itr;
        }
                                
                
        // move to new line.
        switch(_layout)
        {
          case LEFT_TO_RIGHT:
          {
            startOfLine_coords.y() -= _characterHeight * (1.0 + _lineSpacing);
            cursor = startOfLine_coords;
            previous_charcode = 0;
            _lineCount++;
            break;
          }
          case RIGHT_TO_LEFT:
          {
            startOfLine_coords.y() -= _characterHeight * (1.0 + _lineSpacing);
            cursor = startOfLine_coords;
            previous_charcode = 0;
            _lineCount++;
            break;
          }
          case VERTICAL:
          {
            startOfLine_coords.x() += _characterHeight/getCharacterAspectRatio() * (1.0 + _lineSpacing);
            cursor = startOfLine_coords;
            previous_charcode = 0;
            // because _lineCount is the max vertical no. of characters....
            _lineCount = (_lineCount >linelength)?_lineCount:linelength;
          }
          break;
        }
        
        ++lineNumber;
        
        //std::cout << "cursor on line "<< lineNumber << " = " << stringify(cursor) << std::endl;

    }
   
    TextBase::computePositions();
    computeBackdropBoundingBox();
    computeBoundingBoxMargin();
    computeColorGradients();

#else
    osgText::Text::computeGlyphRepresentation();
#endif
#endif

}

osgText::String::iterator spinTextNode::computeLastCharacterOnLine(osg::Vec2& cursor, osgText::String::iterator first, osgText::String::iterator last)
{
#ifdef OSG_MIN_VERSION_REQUIRED
#if OSG_MIN_VERSION_REQUIRED(3,0,0)

    using namespace osgText;
    
    Font* activefont = getActiveFont();
    if (!activefont) return last;

    float hr = _characterHeight;
    float wr = hr/getCharacterAspectRatio();

    bool kerning = true;
    unsigned int previous_charcode = 0;

    String::iterator lastChar = first;

    for(bool outOfSpace=false;lastChar!=last;++lastChar)
    {
        unsigned int charcode = *lastChar;

        if (charcode=='\n')
        {
            return lastChar;
        }

        Glyph* glyph = activefont->getGlyph(_fontSize, charcode);
        if (glyph)
        {

           float width = (float)(glyph->getWidth()) * wr;

            if (_layout==RIGHT_TO_LEFT)
            {
                cursor.x() -= glyph->getHorizontalAdvance() * wr;
            }

            // adjust cursor position w.r.t any kerning.
            if (kerning && previous_charcode)
            {
                switch(_layout)
                {
                  case LEFT_TO_RIGHT:
                  {
                    osg::Vec2 delta(activefont->getKerning(previous_charcode,charcode,_kerningType));
                    cursor.x() += delta.x() * wr;
                    cursor.y() += delta.y() * hr;
                    break;
                  }
                  case RIGHT_TO_LEFT:
                  {
                    osg::Vec2 delta(activefont->getKerning(charcode,previous_charcode,_kerningType));
                    cursor.x() -= delta.x() * wr;
                    cursor.y() -= delta.y() * hr;
                    break;
                  }
                  case VERTICAL:
                    break; // no kerning when vertical.
                }            // check to see if we are still within line if not move to next line.
            }
            
            switch(_layout)
            {
              case LEFT_TO_RIGHT:
              {
                if (_maximumWidth>0.0f && cursor.x()+width>_maximumWidth) outOfSpace=true;
                if(_maximumHeight>0.0f && cursor.y()<-_maximumHeight) outOfSpace=true;
                break;
              }
              case RIGHT_TO_LEFT:
              {
                if (_maximumWidth>0.0f && cursor.x()<-_maximumWidth) outOfSpace=true;
                if(_maximumHeight>0.0f && cursor.y()<-_maximumHeight) outOfSpace=true;
                break;
              }
              case VERTICAL:
                if (_maximumHeight>0.0f && cursor.y()<-_maximumHeight) outOfSpace=true;
                break;
            }

            // => word boundary detection & wrapping
            if (outOfSpace) break;

            // move the cursor onto the next character.
            switch(_layout)
            {
              case LEFT_TO_RIGHT: cursor.x() += glyph->getHorizontalAdvance() * wr; break;
              case VERTICAL:      cursor.y() -= glyph->getVerticalAdvance() *hr; break;
              case RIGHT_TO_LEFT: break; // nop.
            }

            previous_charcode = charcode;

        }

    }
    
    return lastChar;

#else
    return osgText::Text::computeLastCharacterOnLine(cursor, first, last);
#endif
#endif
}


// ===================================================================
// constructor:
TextNode::TextNode (SceneManager *sceneManager, const char* initID) : GroupNode(sceneManager, initID)
{
    using std::string;

	this->setName(this->getID() + ".TextNode");
	this->setNodeType("TextNode");

	text_ = this->getID();
    drawMode_ = GLYPH;
	font_ = "arial.ttf";
	characterSize_ = 0.1f;
    thickness_ = 0.02f;
    resolution_ = 128;
	color_ = osg::Vec4(1.0,1.0,1.0,1.0);
	bgColor_ = osg::Vec4(1.0,0.0,0.0,0.5);
    boxSize_ = osg::Vec2(0,0);
	margin_ = 0.1;
	billboard_ = RELATIVE; // ie, no billboard
	decoration_ = DROP_SHADOW_BOTTOM_RIGHT;
	background_ = NO_BACKGROUND;
    alignment_ = osgText::TextBase::LEFT_BASE_LINE;
    repetitions_ = 1;
    singleSided_ = false;
    lighting_ = false;
	
    updateFlag_ = false;
    redrawFlag_ = false;
    
	// By default osgText is not properly rotated for our use. We want the text
	// to "face" in the direction of the parent's orientation.
	//setOrientation(0,0,180);

	drawText();
}

// ===================================================================
// destructor
TextNode::~TextNode()
{

}

// ===================================================================

void TextNode::callbackUpdate(osg::NodeVisitor* nv)
{
    GroupNode::callbackUpdate(nv);
    
    if (redrawFlag_)
    {
        drawText();
        redrawFlag_ = false;
        updateFlag_ = false;
    }
    else if (updateFlag_)
    {
        updateText();
        updateFlag_ = false;
    }
}

// ===================================================================
// ======================== SET METHODS: =============================
// ===================================================================


void TextNode::setContext (const char *newvalue)
{
	// need to redraw after setContext() is called:
	ReferencedNode::setContext(newvalue);
	//drawText();
    redrawFlag_ = true;
}

void TextNode::setDrawMode (DrawMode mode)
{
	if (mode == drawMode_) return;
	else drawMode_ = mode;
    redrawFlag_ = true;
	BROADCAST(this, "si", "setDrawMode", (int) drawMode_);
}

void TextNode::setText (const char *s)
{
    if (!s)
    {
        std::cout << "WARNING: TextNode::setText got empty string" << std::endl;
        return;
    }
    
	//if (textLabel_->getText().createUTF8EncodedString() != string(s))
	if (text_ != std::string(s))
	{
        text_ = s;
		//getText().createUTF8EncodedString();
		
        /*
		this->_text = string(s);
		pthread_mutex_lock(&sceneMutex);
		textLabel_->setText(s);
		//pthread_mutex_unlock(&sceneMutex);
		//drawText();
        */
        updateFlag_ = true;


		//std::cout << "debug: setting text label to: " << s << ", getTextValue() reports: " << getTextValue() << std::endl;

		BROADCAST(this, "ss", "setText", getText());
	}
}

void TextNode::setRepetitions (int repetitions)
{
	if (repetitions_ != repetitions)
	{
        repetitions_ = repetitions;
        updateFlag_ = true;
		BROADCAST(this, "si", "setRepetitions", getRepetitions());
	}
}

void TextNode::setFont (const char *s)
{
    using std::string;
	if (this->font_ != string(s))
	{
        this->font_ = string(s);
        /*
		pthread_mutex_lock(&sceneMutex);
		textLabel_->setFont( sceneManager_->resourcesPath + "/fonts/" + font_ );
		pthread_mutex_unlock(&sceneMutex);
		//drawText();
        */
        updateFlag_ = true;

		BROADCAST(this, "ss", "setFont", getFont());
	}
}

void TextNode::setFontResolution (int resolution)
{
	if (resolution_ != resolution)
	{
        resolution_ = resolution;
        updateFlag_ = true;
		BROADCAST(this, "si", "setFontResolution", getFontResolution());
	}
}

void TextNode::setCharacterSize (float s)
{
	characterSize_ = s;
	//textLabel_->setCharacterSize( characterSize_ );
    updateFlag_ = true;
	BROADCAST(this, "sf", "setCharacterSize", getCharacterSize());
}

void TextNode::setThickness (float thickness)
{
	thickness_ = thickness;
    updateFlag_ = true;
	BROADCAST(this, "sf", "setThickness", getThickness());
}

void TextNode::setBoxSize (float width, float height)
{
    /*
    pthread_mutex_lock(&sceneMutex);
	textLabel_->setMaximumWidth(width);
    textLabel_->setMaximumHeight(height);
    pthread_mutex_unlock(&sceneMutex);
    */
    boxSize_ = osg::Vec2(width,height);
    updateFlag_ = true;
	BROADCAST(this, "sff", "setBoxSize", boxSize_.x(), boxSize_.y());
}

void TextNode::setLineSpacing (float spacing)
{
    lineSpacing_ = spacing;
    /*
    pthread_mutex_lock(&sceneMutex);
    textLabel_->setLineSpacing(spacing);
   	pthread_mutex_unlock(&sceneMutex);
    BROADCAST(this, "sf", "setLineSpacing", textLabel_->getLineSpacing());
    */
    updateFlag_ = true;
    BROADCAST(this, "sf", "setLineSpacing", getLineSpacing());
}

void TextNode::setAlignment (int alignment)
{
    alignment_ = (osgText::TextBase::AlignmentType)alignment;

    pthread_mutex_lock(&sceneMutex);
    textLabel_->setAlignment((osgText::TextBase::AlignmentType)alignment);
    pthread_mutex_unlock(&sceneMutex);
   	BROADCAST(this, "si", "setAlignment", (int)textLabel_->getAlignment());    
}

void TextNode::setColor (float r, float g, float b, float a)
{
	color_ = osg::Vec4(r,g,b,a);

	if (textLabel_.valid())
	{
#ifdef OSG_MIN_VERSION_REQUIRED
#if OSG_MIN_VERSION_REQUIRED(3,0,0)
		textLabel_->setColor( color_ );
#else
		osgText::Text* tt = dynamic_cast<osgText::Text*>(textLabel_.get());
		if (tt) tt->setColor( color_ );
#endif
#endif
	}
	else
	{
		redrawFlag_ = true;
	}

	BROADCAST(this, "sffff", "setColor", r, g, b, a);
}

void TextNode::setBgColor (float r, float g, float b, float a)
{
	bgColor_ = osg::Vec4(r,g,b,a);
	
#ifdef OSG_MIN_VERSION_REQUIRED
#if OSG_MIN_VERSION_REQUIRED(2,9,7)
	if (textLabel_.valid())
	{
		textLabel_->setBoundingBoxColor(bgColor_);
	}
	else
		redrawFlag_ = true;
#endif
#endif
	BROADCAST(this, "sffff", "setBgColor", r, g, b, a);
}

void TextNode::setMargin (float margin)
{
	margin_ = margin;
#ifdef OSG_MIN_VERSION_REQUIRED
#if OSG_MIN_VERSION_REQUIRED(2,9,7)
	if (textLabel_.valid() ) updateFlag_ = true;
    else redrawFlag_ = true;
#endif
#endif
	BROADCAST(this, "sf", "setMargin", getMargin());
}

void TextNode::setBillboard (billboardType t)
{
	if (t == billboard_) return;
	else billboard_ = t;
    redrawFlag_ = true;
	BROADCAST(this, "si", "setBillboard", (int) billboard_);
}

void TextNode::setDecoration (decorationType t)
{
	decoration_ = t;
	//textLabel_->setBackdropType((osgText::Text::BackdropType)decoration_);
    updateFlag_ = true;
	BROADCAST(this, "si", "setDecoration", getDecoration());
}

void TextNode::setBackground (backgroundType t)
{
	background_ = t;
    updateFlag_ = true;
	BROADCAST(this, "si", "setBackground", getBackround());

}

void TextNode::setSingleSided (int singleSided)
{
    singleSided_ = singleSided;
    updateFlag_ = true;
	BROADCAST(this, "si", "setSingleSided", getSingleSided());
}

void TextNode::setLighting (int lighting)
{
    lighting_ = lighting;
    
    if (textLabel_.valid())
    {
        osg::StateSet *labelStateSet = textLabel_->getOrCreateStateSet();
        if (lighting_)
        {
            labelStateSet->setMode( GL_LIGHTING, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE );
        } else
        {
            labelStateSet->setMode( GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE );
        }
    }
    else
        redrawFlag_ = true;
    
    BROADCAST(this, "si", "setLighting", getLighting());
}

// =============================================================================
void TextNode::drawText()
{
    using std::string;
	osg::Billboard *b;
	
    //pthread_mutex_lock(&sceneMutex);
	
	
	// first remove existing text:
	if (this->getAttachmentNode()->containsNode(textGeode_.get()))
	{
		this->getAttachmentNode()->removeChild(textGeode_.get());
		textGeode_ = NULL;
	}
    if (textLabel_.valid()) textLabel_ = NULL;

	//bool ignoreOnThisHost = (not spinApp::Instance().getContext()->isServer() && (this->getContext()==getHostname()));

	bool drawOnThisHost = ((this->getContextString() == spinApp::Instance().getUserID()) or
	                       (this->getContextString() == "NULL"));
	
	if (drawOnThisHost)
	{
		if (billboard_)
		{
			b = new osg::Billboard();
			switch (billboard_)
			{
				case POINT_EYE:
					b->setMode(osg::Billboard::POINT_ROT_EYE);
					break;
				case STAY_UP:
					b->setMode(osg::Billboard::AXIAL_ROT);
					b->setAxis(osg::Vec3(0.0f,0.0f,1.0f));
					b->setNormal(osg::Vec3(0.0f,-1.0f,0.0f));
					break;
                default:
                    break;
			}
			textGeode_ = b;
			
		} else {
			textGeode_ = new osg::Geode();
		}
		textGeode_->setName(this->getID() + ".TextGeode");
		
		// attach geode and textLabel_:
		this->getAttachmentNode()->addChild(textGeode_.get());
        
        if (drawMode_==TEXT3D)
        {
            textLabel_ = new osgText::Text3D();
        }
        else
        {
            spinTextNode *n = new spinTextNode();
            textLabel_ = n;
            //testLabel_ = new osgText::Text();
#ifdef OSG_MIN_VERSION_REQUIRED
#if OSG_MIN_VERSION_REQUIRED(3,0,0)
            n->setEnableDepthWrites(true);
#endif
#endif
        }
        
        osg::StateSet *labelStateSet = textLabel_->getOrCreateStateSet();
        
        
        textLabel_->setLayout(osgText::TextBase::LEFT_TO_RIGHT);
        
		textGeode_->addDrawable(textLabel_.get());
        
        
		// allow transparency:
		labelStateSet->setMode( GL_BLEND, osg::StateAttribute::ON );
		labelStateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
        
        if (singleSided_)
            labelStateSet->setMode( GL_CULL_FACE, osg::StateAttribute::ON );
        else
            labelStateSet->setMode( GL_CULL_FACE, osg::StateAttribute::OFF );
        
        labelStateSet->setMode( GL_DEPTH_TEST, osg::StateAttribute::ON );
        
        if (lighting_)
        {
            labelStateSet->setMode( GL_LIGHTING, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE );
        } else
        {
            labelStateSet->setMode( GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE );
        }
        
        labelStateSet->setRenderBinDetails( 100, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
        
        updateText();
    }
}

void TextNode::updateText()
{
    if (textLabel_.valid())
    {
		// set some parameters for the text:
		textLabel_->setCharacterSize(characterSize_);
		//textLabel_->setFont(0); // inbuilt font (small)
		//textLabel_->setFont( sceneManager_->resourcesPath + "/fonts/" + font_ );
#ifdef OSG_MIN_VERSION_REQUIRED
#if OSG_MIN_VERSION_REQUIRED(3,0,0)
		textLabel_->setFont( font_ );
#else
		osgText::Text* tt = dynamic_cast<osgText::Text*>(textLabel_.get());
		if (tt) tt->setFont( font_ );
#endif
#endif
		//textLabel_->setFontResolution(40,40);
        //textLabel_->setFontResolution(80,80);
        textLabel_->setFontResolution(resolution_,resolution_);

#ifdef OSG_MIN_VERSION_REQUIRED
#if OSG_MIN_VERSION_REQUIRED(3,0,0)
		textLabel_->setColor( color_ );
#else
		tt = dynamic_cast<osgText::Text*>(textLabel_.get());
		if (tt) tt->setColor( color_ );
#endif
#endif
	

        
#ifdef OSG_MIN_VERSION_REQUIRED
#if OSG_MIN_VERSION_REQUIRED(2,9,7)
		textLabel_->setBoundingBoxColor(bgColor_);
		textLabel_->setBoundingBoxMargin(margin_);
#endif
#endif
        if (drawMode_==GLYPH)
        {
            osgText::Text *t = dynamic_cast<osgText::Text*>(textLabel_.get());
            if (t) t->setBackdropType((osgText::Text::BackdropType)decoration_);
        }
        
        if (drawMode_==TEXT3D)
        {
            osgText::Text3D *t = dynamic_cast<osgText::Text3D*>(textLabel_.get());
            if (t) t->setCharacterDepth(thickness_);
        }

        float boxScalar = 1.0;
        if ((drawMode_==TEXT3D) && (characterSize_!=0)) boxScalar = 1/characterSize_;
        textLabel_->setMaximumWidth(boxSize_.x() * boxScalar);
        textLabel_->setMaximumHeight(boxSize_.y() * boxScalar);

		// setDrawMode (background):
		if (background_ == NO_BACKGROUND)
			textLabel_->setDrawMode(osgText::Text::TEXT);
		else if (background_ == FILLED)
        {
#ifdef OSG_MIN_VERSION_REQUIRED
#if OSG_MIN_VERSION_REQUIRED(2,9,7)
			textLabel_->setDrawMode(osgText::Text::TEXT | osgText::Text::FILLEDBOUNDINGBOX);
#else
			textLabel_->setDrawMode(osgText::Text::TEXT);
#endif
#endif
        }
		else if (background_ == WIREFRAME)
			textLabel_->setDrawMode(osgText::Text::TEXT | osgText::Text::BOUNDINGBOX);
		else if (background_ == ALL)
        {
#ifdef OSG_MIN_VERSION_REQUIRED
#if OSG_MIN_VERSION_REQUIRED(2,9,7)
			textLabel_->setDrawMode(osgText::Text::TEXT | osgText::Text::FILLEDBOUNDINGBOX | osgText::Text::BOUNDINGBOX);
#else
			textLabel_->setDrawMode(osgText::Text::TEXT | osgText::Text::BOUNDINGBOX);
#endif
#endif
        }


		// setAlignment
		// LEFT_TOP, LEFT_CENTER, LEFT_BOTTOM, CENTER_TOP,
		// CENTER_CENTER, CENTER_BOTTOM, RIGHT_TOP, RIGHT_CENTER,
		// RIGHT_BOTTOM, LEFT_BASE_LINE, CENTER_BASE_LINE, RIGHT_BASE_LINE,
		// LEFT_BOTTOM_BASE_LINE, CENTER_BOTTOM_BASE_LINE, RIGHT_BOTTOM_BASE_LINE
		textLabel_->setAlignment(alignment_);

		//textLabel_->setRotation(osg::Quat(osg::PI_2, osg::X_AXIS) * osg::Quat(osg::PI, osg::Z_AXIS));
		textLabel_->setRotation(osg::Quat(osg::PI_2, osg::X_AXIS));

        textLabel_->setLineSpacing(lineSpacing_);

		
        
        
        // finally, set the actual text string:
        std::string finalString;
        for (int i=0; i<repetitions_; i++) finalString += text_;
        textLabel_->setText(finalString, osgText::String::ENCODING_UTF8);
	}

	//pthread_mutex_unlock(&sceneMutex);

}

// =============================================================================
std::vector<lo_message> TextNode::getState () const
{
	// inherit state from base class
	std::vector<lo_message> ret = GroupNode::getState();

	lo_message msg;
	osg::Vec4 v4;

	msg = lo_message_new();
	lo_message_add(msg, "si", "setDrawMode", getDrawMode());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "si", "setRepetitions", getRepetitions());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "ss", "setText", getText());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "ss", "setFont", getFont());
	ret.push_back(msg);
    
	msg = lo_message_new();
	lo_message_add(msg,  "si", "setFontResolution", getFontResolution());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "sf", "setCharacterSize", getCharacterSize());
	ret.push_back(msg);
    
	msg = lo_message_new();
	lo_message_add(msg, "sf", "setThickness", getThickness());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "sff", "setBoxSize", boxSize_.x(), boxSize_.y());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "sf", "setLineSpacing", getLineSpacing());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "si", "setAlignment", (int)textLabel_->getAlignment());
	ret.push_back(msg);

	msg = lo_message_new();
	v4 = this->getColor();
	lo_message_add(msg, "sffff", "setColor", v4.x(), v4.y(), v4.z(), v4.w());
	ret.push_back(msg);

	msg = lo_message_new();
	v4 = this->getBgColor();
	lo_message_add(msg, "sffff", "setBgColor", v4.x(), v4.y(), v4.z(), v4.w());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "sf", "setMargin", getMargin());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "si", "setBillboard", getBillboard());
	ret.push_back(msg);
	
	msg = lo_message_new();
	lo_message_add(msg, "si", "setDecoration", getDecoration());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "si", "setBackground", getBackround());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "si", "setSingleSided", getSingleSided());
	ret.push_back(msg);
 
	msg = lo_message_new();
	lo_message_add(msg, "si", "setLighting", getLighting());
	ret.push_back(msg);
   
	return ret;
}

} // end of namespace spin

