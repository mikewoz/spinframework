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

// NOTE: this code is originally from osgBullet (LGPL)
// osgBullet is (C) Copyright 2009-2011 by Kenneth Mark Bryden

#ifndef __bulletUtil_H
#define __bulletUtil_H


#include <osg/Matrix>
#include <osg/Vec3>
#include <osg/Vec4>
#include <osg/Quat>

#include <LinearMath/btTransform.h>


namespace spin
{


/** \defgroup conversionutils Vector and Matrix Data Conversion Utilities
\brief Convenience functions for converting between OSG's and Bullet's vector and matrix classes.
*/
/**@{*/

osg::Matrix asOsgMatrix( const btTransform& t );
btTransform asBtTransform( const osg::Matrix& m );

osg::Matrix asOsgMatrix( const btMatrix3x3& m );
btMatrix3x3 asBtMatrix3x3( const osg::Matrix& m );

osg::Vec3 asOsgVec3( const btVector3& v );
btVector3 asBtVector3( const osg::Vec3& v );

osg::Vec4 asOsgVec4( const btVector3& v, const double w );
osg::Vec4 asOsgVec4( const btVector4& v );
osg::Quat asOsgQuat( const btQuaternion& q); // mikewoz
btQuaternion asBtQuaternion( const osg::Quat& q); // mikewoz
btVector4 asBtVector4( const osg::Vec4& v );

/**@{*/
}

#endif