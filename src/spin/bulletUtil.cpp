// NOTE: this code is originally from osgBullet (LGPL)
// osgBullet is (C) Copyright 2009-2011 by Kenneth Mark Bryden

#include <osg/Matrix>
#include <LinearMath/btTransform.h>

#include "bulletUtil.h"

//using namespace spin;

// Convert a btTransform to an OSG Matrix
osg::Matrix spin::asOsgMatrix( const btTransform& t )
{
    btScalar ogl[ 16 ];
    t.getOpenGLMatrix( ogl );
    osg::Matrix m( ogl );
    return m;
}

btTransform spin::asBtTransform( const osg::Matrix& m )
{
    const osg::Matrix::value_type* oPtr = m.ptr();
    btScalar bPtr[ 16 ];
    int idx;
    for (idx=0; idx<16; idx++)
        bPtr[ idx ] = oPtr[ idx ];
    btTransform t;
    t.setFromOpenGLMatrix( bPtr );
    return t;
}


osg::Matrix spin::asOsgMatrix( const btMatrix3x3& m )
{
    btScalar f[ 9 ];
    m.getOpenGLSubMatrix( f );
    return( osg::Matrix(
        f[0], f[1], f[2], 0.,
        f[3], f[4], f[5], 0.,
        f[6], f[7], f[8], 0.,
        0., 0., 0., 1. ) );
}

btMatrix3x3 spin::asBtMatrix3x3( const osg::Matrix& m )
{
    return( btMatrix3x3(
        m(0,0), m(0,1), m(0,2),
        m(1,0), m(1,1), m(1,2),
        m(2,0), m(2,1), m(2,2) ) );
}

osg::Vec3 spin::asOsgVec3( const btVector3& v )
{
    return osg::Vec3( v.x(), v.y(), v.z() );
}

btVector3 spin::asBtVector3( const osg::Vec3& v )
{
    return btVector3( v.x(), v.y(), v.z() );
}

osg::Vec4 spin::asOsgVec4( const btVector3& v, const double w )
{
    return osg::Vec4( v.x(), v.y(), v.z(), w );
}

osg::Vec4 spin::asOsgVec4( const btVector4& v )
{
    return osg::Vec4( v.x(), v.y(), v.z(), v.w() );
}

osg::Quat spin::asOsgQuat( const btQuaternion& q)
{
    return osg::Quat( q.getAngle(), spin::asOsgVec3(q.getAxis()) );
}

btQuaternion spin::asBtQuaternion( const osg::Quat& q)
{
    return btQuaternion( q.x(), q.y(), q.z(), q.w() );
}

btVector4 spin::asBtVector4( const osg::Vec4& v )
{
    return btVector4( v.x(), v.y(), v.z(), v.w() );
}
