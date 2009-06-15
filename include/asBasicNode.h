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


#ifndef __ASBASICNODE_H
#define __ASBASICNODE_H

#include "asGlobals.h"
#include "asReferenced.h"

#include <osg/Group>
#include <osg/PositionAttitudeTransform> 
#include <osg/NodeVisitor>

#include <string>
#include <vector>
#include <map>

/**
 * \brief A basic node to manage translation/orientation/scale of a subgraph.
 *        Allows for grouping of nodes.
 * 
 * Most nodes in the scene do not have a method to offset their position in the
 * scene. This node provides a mechanism to group and offset nodes.
 */
class asBasicNode : public asReferenced
{

public:
	
	/**
	 * The constructor takes a string id that will be converted to a t_symbol,
	 * which contains a pointer to this object. Thus functions will be able to
	 * acquire the pointer using gensym(), which uses a hash table lookup.
	 * @param initID will be converted into a t_symbol
	 */

	asBasicNode(asSceneManager *sceneManager, char *initID);
	virtual ~asBasicNode();
	
	virtual void callbackUpdate();

	
	/**
	 * IMPORTANT:
	 * subclasses of asReferenced are allowed to contain complicated subgraphs,
	 * and can also change their attachmentNode so that children are attached
	 * anywhere in this subgraph. If that is the case, the updateNodePath()
	 * function MUST be overridden, and extra nodes must be manually pushed onto
	 * the currentNodePath.
	 */
	virtual void updateNodePath();
	
	void reportGlobals (int b);
	void setTranslation (float x, float y, float z);
	void setOrientation (float p, float r, float y);
	void move (float x, float y, float z);
	void rotate (float p, float r, float y);
	
	int getReportGlobals() { return (int)_reportGlobals; };
	osg::Vec3 getTranslation() { return mainTransform->getPosition(); };
	osg::Vec3 getOrientation() { return _orientation; };
	//osg::Vec3 getOrientation() { return Vec3inDegrees((mainTransform->getAttitude()).asVec3()); };
	
	
	// extra method for globals:
	void dumpGlobals(bool forced);
	
	
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
	virtual void stateDump() { asReferenced::stateDump(); dumpGlobals(true); };


	
	// ***********************************************************
	// data:
	
	osg::ref_ptr<osg::PositionAttitudeTransform> mainTransform;

	
private:
	bool _reportGlobals;
	osg::Vec3 _orientation; // store the orientation as it comes in (in degrees)
	osg::Matrix _globalMatrix;
	osg::Vec3 _globalScale;
	float _globalRadius;
		
};


	
#endif
