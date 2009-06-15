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


#ifndef __OSGUTIL_H
#define __OSGUTIL_H

#include <iostream>

#include <osg/Vec3>
#include <osg/Quat>
#include <osg/Matrix>
#include <osg/PositionAttitudeTransform>
#include <osg/Group>
#include <osg/Geometry>
#include <osg/Geode>



#ifdef __APPLE__
	#if !defined(isnan)
		#include <math.h> // dont_vxl_filter: this is *not* supposed to be <cmath>
		#define isnan(x) __inline_isnand((double)x)
	#endif
#endif


#include "asReferenced.h"

#define Vec3inDegrees(v) (osg::Vec3( osg::RadiansToDegrees(v.x()), osg::RadiansToDegrees(v.y()), osg::RadiansToDegrees(v.z()) ))

#define X_AXIS osg::Vec3(1.0, 0.0, 0.0)
#define Y_AXIS osg::Vec3(0.0, 1.0, 0.0)
#define Z_AXIS osg::Vec3(0.0, 0.0, 1.0)


double AngleBetweenVectors(osg::Vec3 v1, osg::Vec3 v2);
osg::Vec3 rotateAroundAxis(osg::Vec3 v, osg::Vec3 axis, float angle);
osg::Quat EulerToQuat(float roll, float pitch, float yaw);
osg::Vec3 QuatToEuler(osg::Quat q);

osg::Geode* 	createGrid(int radius, osg::Vec4 color);
osg::Geometry*	createPlane(float halfLength, osg::Vec4 color);
osg::Geode*		createHollowSphere(float radius, osg::Vec4 color);
osg::Geode*		createWireframeRolloff(int rolloff, float distortion, float scale, osg::Vec4 color);
osg::Geode*		createHollowCone(float length, float radius, osg::Vec4 color);


// can't make this work (argh!):
/*
class worldMatrixUpdater : public osg::NodeVisitor
{
	public:
		
		worldMatrixUpdater() : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN) {}
		
		virtual void apply(osg::PositionAttitudeTransform &node)
	  {

			ss_soundNode *ourNode = dynamic_cast<ss_soundNode*>(node.getUserData());
			osg::Matrix mat = osg::computeWorldToLocal( getNodePath() );
			osg::Vec3 trans = mat.getTrans();
			osg::notify(osg::NOTICE) << "global position of '" << ourNode->id->s_name << "' is (" << trans.x() << "," << trans.y() << "," << trans.z() << ")" << std::endl;
			//ourNode->worldMatrix = osg::computeWorldToLocal( getNodePath() );
			traverse(node);

			
			//for (unsigned int i = 0; i< node.getNumChildren(); i++)
			//{
			//	osg::ref_ptr<osg::PositionAttitudeTransform> nd = dynamic_cast<osg::PositionAttitudeTransform*>( node.getChild(i) );
			//	if ( nd.valid() ) traverse(*(nd.get()));
			//}
			
		}
		
		virtual void apply(osg::Node &node) { traverse(node); }		 
		virtual void apply(osg::Geode &node) { traverse(node); }
		virtual void apply(osg::Group &node) { traverse(node); }
		
};
*/




#endif
