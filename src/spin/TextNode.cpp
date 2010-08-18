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
#include <osg/Billboard>

#include "osgUtil.h"
#include "TextNode.h"
#include "SceneManager.h"
#include "spinApp.h"
#include "spinBaseContext.h"
#include "MediaManager.h"



using namespace std;


extern pthread_mutex_t sceneMutex;


// ===================================================================
// constructor:
TextNode::TextNode (SceneManager *sceneManager, char *initID) : GroupNode(sceneManager, initID)
{
	this->setName(string(id->s_name) + ".TextNode");
	nodeType = "TextNode";

	//_text = "";
	_font = "arial.ttf";
	_color = osg::Vec4(1.0,1.0,1.0,1.0);
	_billboard = RELATIVE; // ie, no billboard

	textLabel = new osgText::Text();
	
	// By default osgText is not properly rotated for our use. We want the text
	// to "face" in the direction of the parent's orientation.
	setOrientation(0,0,180);

	drawText();
}

// ===================================================================
// destructor
TextNode::~TextNode()
{

}

// ===================================================================

// ===================================================================
// ======================== SET METHODS: =============================
// ===================================================================


void TextNode::setContext (const char *newvalue)
{
	// need to redraw after setContext() is called:
	ReferencedNode::setContext(newvalue);
	drawText();
}

void TextNode::setTextValue (const char *s)
{
	//if (textLabel->getText().createUTF8EncodedString() != string(s))
	if (_text != string(s))
	{
		//getText().createUTF8EncodedString();
		
		this->_text = string(s);
		pthread_mutex_lock(&sceneMutex);
		textLabel->setText(s);
		pthread_mutex_unlock(&sceneMutex);
		//drawText();

		//std::cout << "debug: setting text label to: " << s << ", getTextValue() reports: " << getTextValue() << std::endl;

		BROADCAST(this, "ss", "setTextValue", getTextValue());
	}
}

void TextNode::setFont (const char *s)
{
	if (this->_font != string(s))
	{
		this->_font = string(s);
		pthread_mutex_lock(&sceneMutex);
		textLabel->setFont( sceneManager->resourcesPath + "/fonts/" + _font );
		pthread_mutex_unlock(&sceneMutex);
		//drawText();

		BROADCAST(this, "ss", "setFont", getFont());
	}
}

void TextNode::setBillboard (billboardType t)
{
	if (t == _billboard) return;
	else _billboard = t;

	drawText();

	BROADCAST(this, "si", "setBillboard", (int) _billboard);

}

void TextNode::setColor (float r, float g, float b, float a)
{
	_color = osg::Vec4(r,g,b,a);

	textLabel->setColor( _color );
	//drawText();

	BROADCAST(this, "sffff", "setColor", r, g, b, a);
}

// =============================================================================
void TextNode::drawText()
{

    pthread_mutex_lock(&sceneMutex);

	// first remove existing text:
	if (this->getAttachmentNode()->containsNode(textGeode.get()))
	{
		this->getAttachmentNode()->removeChild(textGeode.get());
		textGeode = NULL;
	}

	//bool ignoreOnThisHost = (not spinApp::Instance().getContext()->isServer() && (this->getContext()==getHostname()));

	bool drawOnThisHost = ((this->getContextString() == spinApp::Instance().getUserID()) or
	                       (this->getContextString() == "NULL"));
	
	if (drawOnThisHost)
	{
		if (_billboard)
		{
			osg::Billboard *b = new osg::Billboard();
			switch (_billboard)
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
			textGeode = b;
			
		} else {
			textGeode = new osg::Geode();
		}
		textGeode->setName(string(id->s_name) + ".textGeode");
		
		// attach geode and textLabel:
		this->getAttachmentNode()->addChild(textGeode.get());
		
		textGeode->addDrawable(textLabel.get());


		// set some parameters for the text:
		textLabel->setCharacterSize(0.1f);
		//textLabel->setFont(0); // inbuilt font (small)
		textLabel->setFont( sceneManager->resourcesPath + "/fonts/" + _font );
		textLabel->setFontResolution(40,40);
		textLabel->setColor( _color );

		// setDrawMode
		// TEXT = 1, BOUNDINGBOX = 2, ALIGNMENT = 4
		//textLabel->setDrawMode(osgText::Text::TEXT); 

		// setAlignment
		// LEFT_TOP, LEFT_CENTER, LEFT_BOTTOM, CENTER_TOP,
		// CENTER_CENTER, CENTER_BOTTOM, RIGHT_TOP, RIGHT_CENTER,
		// RIGHT_BOTTOM, LEFT_BASE_LINE, CENTER_BASE_LINE, RIGHT_BASE_LINE,
		// LEFT_BOTTOM_BASE_LINE, CENTER_BOTTOM_BASE_LINE, RIGHT_BOTTOM_BASE_LINE
		textLabel->setAlignment(osgText::Text::CENTER_CENTER);

		//textLabel->setRotation(osg::Quat(osg::PI_2, osg::X_AXIS) * osg::Quat(osg::PI, osg::Z_AXIS));
		textLabel->setRotation(osg::Quat(osg::PI_2, osg::X_AXIS));

		
		// disable lighting effects on the text, and allow transparency:
		osg::StateSet *labelStateSet = new osg::StateSet;
		labelStateSet->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
		labelStateSet->setMode( GL_BLEND, osg::StateAttribute::ON );
		labelStateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
		labelStateSet->setRenderBinDetails( 100, "RenderBin");
		textLabel->setStateSet( labelStateSet );
	}

	pthread_mutex_unlock(&sceneMutex);

}

// =============================================================================
std::vector<lo_message> TextNode::getState ()
{
	// inherit state from base class
	std::vector<lo_message> ret = GroupNode::getState();

	lo_message msg;

	msg = lo_message_new();
	lo_message_add(msg, "ss", "setTextValue", getTextValue());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "ss", "setFont", getFont());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "si", "setBillboard", getBillboard());
	ret.push_back(msg);
	
	msg = lo_message_new();
	osg::Vec4 v4 = this->getColor();
	lo_message_add(msg, "sffff", "setColor", v4.x(), v4.y(), v4.z(), v4.w());
	ret.push_back(msg);

	return ret;
}
