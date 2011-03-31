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

#include <osg/Geometry>
#include <osgUtil/Optimizer>
#include <osg/ShapeDrawable>

#include "Contour.h"
#include "SceneManager.h"
#include "spinApp.h"
#include "spinBaseContext.h"
#include "osgUtil.h"

using namespace std;

//extern SceneManager *sceneManager;


namespace spin
{


// *****************************************************************************
// constructor:
Contour::Contour (SceneManager *sceneManager, char *initID) : ReferencedNode(sceneManager, initID)
{
	this->setName(string(id->s_name) + ".Contour");
	nodeType = "Contour";

	_vArray = new osg::Vec3Array;
	_vArray->clear();
	
	_maxVertices= 100;
	_currentIndex = 0;

	_visible = true;
	_color = osg::Vec4(0,1,0,1); //green
	
	_thickness = 0.1;
	_lineType = THIN;
	
	_trackingMode = POSITION;
	
	
	vArrayGeode = NULL;
	//vArrayGeometry = NULL;

	mainTransform = new osg::PositionAttitudeTransform();
	mainTransform->setName(string(id->s_name) + ".mainTransform");
	this->addChild(mainTransform.get());
	
	// When children are attached to this, they get added to the attachNode:
	// NOTE: by changing this, we MUST override the updateNodePath() method!
	setAttachmentNode(mainTransform.get());
	
	
	// for testing:
	/*
	add(0,0,0);
	add(1,0,0);
	add(1,1,0);
	add(1,1,1);
	add(-1,1,0);
	add(-1,1,1);
	add(1,1,0);
	add(-1,1,0);
	add(-1,0,0);
	*/

	_redrawFlag = true;
}

// *****************************************************************************
// destructor
Contour::~Contour()
{
	std::cout << " GOT TO Contour DESTRUCTOR !!! " << std::endl;
	// TODO: uh oh !! we never get here. This is a potential memory leak.

	_vArray->clear();
}


void Contour::updateNodePath()
{
	currentNodePath.clear();
	if ((parent!=WORLD_SYMBOL) && (parent!=NULL_SYMBOL))
	{
		osg::ref_ptr<ReferencedNode> parentNode = dynamic_cast<ReferencedNode*>(parent->s_thing);
		if (parentNode.valid())
		{
			currentNodePath = parentNode->currentNodePath;
		}
	}
	
	// here, the nodePath includes the base osg::group, PLUS the mainTransform
	currentNodePath.push_back(this);
	currentNodePath.push_back(mainTransform.get());
	
	// now update NodePaths for all children:
	updateChildNodePaths();
	
}

void Contour::callbackUpdate()
{
	if (_redrawFlag) draw();
}


// *****************************************************************************
void Contour::updateTransforms()
{
	mainTransform->setPosition(getTranslation(_currentIndex));
	if (_trackingMode==FULL6DOF)
	{
		mainTransform->setAttitude(getOrientation((int)_currentIndex));
	}
}


// *****************************************************************************
// This function gets the orientation of a given index into the soundLine. Note
// that the orientation along a line has only 2 degrees of freedom (roll is not
// defined). So we specify the Y_AXIS.
osg::Quat Contour::getOrientation(int index) const
{
	if (_vArray->size() < 2)
	{
		return osg::Quat();
	}
	else if (index == _vArray->size() - 1)
	{
		return getOrientation(index-1);
	}
	
	else
	{
		
		osg::Quat rot;
		osg::Vec3 v = osg::Vec3(_vArray->at((int)index+1)-_vArray->at((int)index));
		//rot.makeRotate( osg::X_AXIS, v )
		
		v.normalize();
		//rot.makeRotate(0.0, v);
		//rot.makeRotate( (float)v.y(), osg::X_AXIS, (float)0.0,osg:: Y_AXIS, (float)v.x(), osg::Z_AXIS );
		rot.makeRotate( osg::Z_AXIS, v);
		
		return rot;
	}
}

// We allow the index to be a float, so position can be interpolated between
// two indices:
osg::Vec3 Contour::getTranslation(float index) const
{
	
	if (!_vArray->size())
	{
		return osg::Vec3(0,0,0);
	}
	
	else if ((int)index >= _vArray->size()-1)
	{
		return _vArray->at(_vArray->size()-1);
	}
	
	else
	{
		float remainder = index - (int)index;
		osg::Vec3 dirVector = osg::Vec3(_vArray->at((int)index+1)-_vArray->at((int)index));
		if ((remainder) && (dirVector.length()))
		{
			return osg::Vec3( _vArray->at((int)index) + (dirVector * remainder) );
		}
		else return _vArray->at((int)index);
	}
}




// *****************************************************************************

void Contour::setCurrentIndex (float newValue)
{
	// note newValue is in range [0,1] so we must scale:
	//_currentIndex = (int) newValue * _vArray->size();
	
	if (newValue<0) _currentIndex = 0;
	else if (newValue>_vArray->size()-1) _currentIndex=_vArray->size()-1;
	else _currentIndex = newValue;
	
	updateTransforms();
	
	BROADCAST(this, "sf", "setCurrentIndex", _currentIndex);
}

void Contour::next()
{
	_currentIndex = _currentIndex++;
	if (_currentIndex > _vArray->size()-1) _currentIndex = 0;
	
	updateTransforms();
	
	BROADCAST(this, "sf", "setCurrentIndex", _currentIndex);
}	
void Contour::prev()
{
	_currentIndex = _currentIndex--;
	if (_currentIndex < 0) _currentIndex = _vArray->size()-1;
	
	updateTransforms();
	
	BROADCAST(this, "sf", "setCurrentIndex", _currentIndex);
}




// *****************************************************************************
void Contour::reset()
{
	_currentIndex = 0;
	_vArray->clear();
	updateTransforms();
	
	//std::cout << "cleared the Contour (" << this->id->s_name << ") size=" << _vArray->size() << std::endl;
	
	BROADCAST(this, "s", "reset");
	BROADCAST(this, "sf", "setCurrentIndex", _currentIndex);
	
	_redrawFlag = true;
}

// *****************************************************************************
// We add new values to the front of the vectorArray. If the array grows to more
// than the maxSize, then we pop the last element off the end of the array.
void Contour::add (float x, float y, float z)
{
	if (_maxVertices <= 0) return;
	
	if (_vArray->size() > _maxVertices)
		_vArray->pop_back(); // LIFO
	
	if (!_vArray->size())
		// special case for 1st vertex:
		_vArray->push_back( osg::Vec3(x, y, z) );
	else
		// newest vertex is at beginning of array:
		_vArray->insert( _vArray->begin(), 1, osg::Vec3(x, y, z));

	
	
	// keep index at same position as before the add (means add one to the index
	// since this point gets added to the front of the vArray
	//if (_currentIndex+1 < _vArray->size()) _currentIndex++;
	
	
	updateTransforms();
	_redrawFlag = true;
	
	BROADCAST(this, "sfff", "add", x, y, z);
	//BROADCAST(this, "sf", "setCurrentIndex", _currentIndex);
	
}

void Contour::setMaxVertices (int newMax)
{
	_maxVertices = newMax;
	
	// resize the vectorArray if it is too big:
	if (_vArray->size() > _maxVertices) _vArray->resize(_maxVertices);
	
	// check that _currentIndex is not out of range now:
	if (!_vArray->size()) _currentIndex = 0;
	else if (_currentIndex > _vArray->size()-1) _currentIndex = _vArray->size()-1;
	
	updateTransforms();
	_redrawFlag = true;
	
	BROADCAST(this, "si", "setMaxVertices", _maxVertices);
}

// *****************************************************************************


void Contour::setTrackingMode (int newValue)
{
	_trackingMode = (trackingModeEnum) newValue;
	if (_trackingMode!=FULL6DOF)
	{
		mainTransform->setAttitude(osg::Quat(0,0,0,0));
	}
	
	updateTransforms();

	BROADCAST(this, "si", "setTrackingMode", (int) _trackingMode);
}


void Contour::setVisible (int b)
{
	if (this->_visible != (bool)b)
	{
		this->_visible = (bool) b;
		_redrawFlag = true;
		
		BROADCAST(this, "si", "setVisible", (int) this->_visible);
	}
}

void Contour::setThickness (float newValue)
{
	_thickness = newValue;		
	_redrawFlag = true;

	BROADCAST(this, "sf", "setThickness", _thickness);
}

void Contour::setLineType (int newValue)
{
	_lineType = (ContourTypeEnum) newValue;
	_redrawFlag = true;

	BROADCAST(this, "si", "setLineType", (int) _lineType);
}

void Contour::setColor (float newR, float newG, float newB, float newA)
{
	if (newR<0) newR = 0;
	if (newG<0) newG = 0;
	if (newB<0) newB = 0;
	if (newR>1) newR = 1;
	if (newG>1) newG = 1;
	if (newB>1) newB = 1;

	_color = osg::Vec4(newR, newG, newB, newA);
	
	_redrawFlag = true;
	
	BROADCAST(this, "sffff", "setColor", _color.x(), _color.y(), _color.z(), _color.w());
}




// *****************************************************************************
// ****************************** DRAW METHODS: ********************************
// *****************************************************************************

void Contour::draw()
{
	
	/*
	std::cout << "Drawing Contour (size=" << _vArray->size() << ")" << std::endl;
	osg::Vec3 p;
	for (int j=0; j<_vArray->size(); j++)
	{
		p = _vArray->at(j);
		std::cout << p.x() << " " << p.y() << " " << p.z() << std::endl;
	}
	*/
	
	if (this->containsNode(vArrayGeode.get()))
	{
        this->removeChild(vArrayGeode.get());
	}
	vArrayGeode = NULL;
	
	if ((_visible > 0) && (_vArray->size() > 1))
	{  

		osg::TessellationHints* hints = new osg::TessellationHints;
		hints->setDetailRatio(GENERIC_SHAPE_RESOLUTION);

		vArrayGeode = new osg::Geode();
		osg::ref_ptr<osg::Geometry> vArrayGeometry = new osg::Geometry();
		
		// *************
		// draw vertexArray with simple lines:
		if (_lineType==THIN)
		{
		
			vArrayGeometry->setVertexArray(_vArray.get());
			
			// set the normal:
			//osg::Vec3Array* normals = new osg::Vec3Array;
			//normals->push_back(osg::Vec3(0.0f,-1.0f,0.0f));
			//vArrayGeometry->setNormalArray(normals);
			//vArrayGeometry->setNormalBinding(osg::Geometry::BIND_OVERALL);
			
			//vArrayGeometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,_vArray->size()));
			vArrayGeometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP,0,_vArray->size()));
			 
			// add the geomtry to the geode:
			vArrayGeode->addDrawable( vArrayGeometry.get() );
			
			// set the color:
			osg::Vec4Array* colors = new osg::Vec4Array;
			colors->push_back(_color);
			vArrayGeometry->setColorArray(colors);
			vArrayGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);
		}
		
		// *************
		// draw vertexArray as a series of cylinders:
		else if (_lineType==CYLINDRICAL)
		{
			osg::Cylinder *cyl;
			osg::ShapeDrawable* cylDrawable;
			
			for (int i=0; i<_vArray->size()-1; i++)
			{
				//osg::Cylinder(center, radius, height)
				cyl = new osg::Cylinder(getTranslation(i+0.5), _thickness, (getTranslation(i+1)-getTranslation(i)).length());
				cyl->setRotation(getOrientation(i));// * osg::Quat(osg::PI/2, osg::Y_AXIS)); // * osg::Matrix::rotate(osg::PI/2,0,1,0);
				
				cylDrawable = new osg::ShapeDrawable(cyl,hints);
				cylDrawable->setColor(_color);
				vArrayGeode->addDrawable(cylDrawable);
			}
		}
		
		
		osg::StateSet *vArrayStateSet = new osg::StateSet;
		vArrayStateSet->setMode( GL_LIGHTING, osg::StateAttribute::OFF);			
		vArrayGeode->setStateSet ( vArrayStateSet );
		
		vArrayGeode->setName(string(id->s_name) + ".vArrayGeode");

		this->addChild( vArrayGeode.get() );

	}

}  




// *****************************************************************************
std::vector<lo_message> Contour::getState () const
{
	// inherit state from base class
	std::vector<lo_message> ret = ReferencedNode::getState();
	
	lo_message msg;
	osg::Vec3 v3;
	osg::Vec4 v4;
	
	msg = lo_message_new();
	lo_message_add(msg, "sf", "setCurrentIndex", _currentIndex);
	ret.push_back(msg);	
	
	msg = lo_message_new();
	lo_message_add(msg, "si", "setMaxVertices", _maxVertices);
	ret.push_back(msg);	

	msg = lo_message_new();
	lo_message_add(msg, "si", "setTrackingMode", (int) _trackingMode);
	ret.push_back(msg);	

	msg = lo_message_new();
	lo_message_add(msg, "si", "setVisible", (int) this->_visible);
	ret.push_back(msg);	
	
	msg = lo_message_new();
	lo_message_add(msg, "sf", "setThickness", _thickness);
	ret.push_back(msg);
	
	msg = lo_message_new();
	lo_message_add(msg, "si", "setLineType", (int) _lineType);
	ret.push_back(msg);

	msg = lo_message_new();
	v4 = this->getColor();
	lo_message_add(msg, "sffff", "setColor", v4.x(), v4.y(), v4.z(), v4.w());
	ret.push_back(msg);
	
	for (int i=_vArray->size()-1; i>=0; i--)
	{
		// remember LIFO
		msg = lo_message_new();
		lo_message_add(msg, "sfff", "add", _vArray->at(i).x(), _vArray->at(i).y(), _vArray->at(i).z());
		ret.push_back(msg);	
	}
	
	return ret;
}

} // end of namespace spin

