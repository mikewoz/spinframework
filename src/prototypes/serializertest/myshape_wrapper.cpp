#include <osgDB/ObjectWrapper>
#include <osgDB/InputStream>
#include <osgDB/OutputStream>
#include <osgDB/Registry>

#include "myshape.h"

REGISTER_OBJECT_WRAPPER( MyShape, // The unique wrapper name
                         new mytest::MyShape, // The proto
                         mytest::MyShape, // The class typename
                         "osg::Object osg::Node osg::Group osg::Transform osg::PositionAttitudeTransform mytest::MyShape" )  // The inheritance relations
{
	//ADD_VEC3_SERIALIZER( Translation, osg::Vec3() );
	ADD_STRING_SERIALIZER( Note, "v" );
	ADD_INT_SERIALIZER( Num, 0 );

}

extern "C" void wrapper_serializer_library_mytest(void) {}
