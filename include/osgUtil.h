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


#include "ReferencedNode.h"


namespace spin
{

#define Vec3inDegrees(v) (osg::Vec3( osg::RadiansToDegrees(v.x()), osg::RadiansToDegrees(v.y()), osg::RadiansToDegrees(v.z()) ))


#define GENERIC_SHAPE_RESOLUTION 10.0f

#define AS_UNIT_SCALE  1.0f // 1m
#define AS_DEBUG_SCALE 4.0f // size of debug views (radiation/sensitivity/etc)

/**
 * Returns an absolute angle difference between v1 and v2 (with no notion of
 * which is ahead or behind the other). Returned angle is from 0 to PI
 */
double AngleBetweenVectors(osg::Vec3 v1, osg::Vec3 v2);

/**
 * Returns a signed angle of rotation that describes the rotation from v1 to v2,
 * assuming that one axis is null. 1=X_AXIS, 2=Y_AXIS, 3=Z_AXIS
 */
double AngleBetweenVectors(osg::Vec3 v1, osg::Vec3 v2, int nullAxis);

/**
 * Returns a quaternion that represents the rotation from v1 to v2
 */
osg::Quat RotationBetweenVectors(osg::Vec3 v1, osg::Vec3 v2);


/**
 * Converts a direction vector in cartesian coordinates to radial
 * (spherical) coordinates
 * @return vector containing (azimuth,elevation,distance)
 */
osg::Vec3 cartesianToSpherical(osg::Vec3 v);
    
osg::Vec3 rotateAroundAxis(osg::Vec3 v, osg::Vec3 axis, float angle);
osg::Quat EulerToQuat(float roll, float pitch, float yaw);
osg::Vec3 QuatToEuler(osg::Quat q);
osg::Vec3 QuatToEuler2(osg::Quat q);

//osg::Geode*     createGrid(int radius, osg::Vec4 color);
osg::Geometry*    createPlane(float halfLength, osg::Vec4 color);
osg::Geode*        createHollowSphere(float radius, osg::Vec4 color);
osg::Geode*        createWireframeRolloff(int rolloff, float distortion, float scale, osg::Vec4 color);
osg::Geode*        createHollowCone(float length, float radius, osg::Vec4 color);


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
            //    osg::ref_ptr<osg::PositionAttitudeTransform> nd = dynamic_cast<osg::PositionAttitudeTransform*>( node.getChild(i) );
            //    if ( nd.valid() ) traverse(*(nd.get()));
            //}
            
        }
        
        virtual void apply(osg::Node &node) { traverse(node); }         
        virtual void apply(osg::Geode &node) { traverse(node); }
        virtual void apply(osg::Group &node) { traverse(node); }
        
};
*/


} // end of namespace spin

#endif
