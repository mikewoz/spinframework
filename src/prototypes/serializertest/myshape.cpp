#include <iostream>

#include <osg/Geode>
#include <osg/ShapeDrawable>

#include <osgDB/ObjectWrapper>
#include <osgDB/InputStream>
#include <osgDB/OutputStream>


#include "myshape.h"


REGISTER_OBJECT_WRAPPER( myshape_wrapper, // The unique wrapper name
                         new spinframework::myshape, // The proto
                         spinframework::myshape, // The class typename
                         "osg::Object osg::Node osg::Group osg::Transform osg::PositionAttitudeTransform spinframework::myshape" )  // The inheritance relations
{
	//ADD_VEC3_SERIALIZER( Translation, osg::Vec3() );
	ADD_STRING_SERIALIZER( Note, "v" );
	ADD_INT_SERIALIZER( Num, 0 );

}


using namespace spinframework;

myshape::myshape ()
{
	// let default be a sphere:
	shape_ = myshape::SPHERE;
	note_ = "init";
	num_ = -1;
	init();
}

myshape::myshape (shapeType shape)
{
	shape_ = shape;
	init();
}

myshape::~myshape(){}


void myshape::init()
{
	osg::TessellationHints* hints = new osg::TessellationHints;
	hints->setDetailRatio(0.5f);

	osg::ShapeDrawable *shapeDrawable;
	if (shape_==SPHERE)
	{
		shapeDrawable = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.0f,0.0f,0.0f), SHAPE_SCALE*.5), hints);
	}
	else if (shape_==BOX)
	{
		shapeDrawable = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0.0f,0.0f,0.0f), SHAPE_SCALE), hints);
	}
	else if (shape_==CYLINDER)
	{
		shapeDrawable = new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3(0.0f,0.0f,0.0f), SHAPE_SCALE*.25,SHAPE_SCALE), hints);
	}
	else if (shape_==CAPSULE)
	{
		shapeDrawable = new osg::ShapeDrawable(new osg::Capsule(osg::Vec3(0.0f,0.0f,0.0f), SHAPE_SCALE*.25,SHAPE_SCALE), hints);
	}
	else if (shape_==CONE)
	{
		shapeDrawable = new osg::ShapeDrawable(new osg::Cone(osg::Vec3(0.0f,0.0f,0.0f), SHAPE_SCALE*.25,SHAPE_SCALE), hints);
	}
	else return;

	osg::Geode* geode = new osg::Geode();
	geode->addDrawable(shapeDrawable);
	this->addChild(geode);

}

//void myshape::setTranslation(float x, float y, float z)
void myshape::setTranslation(osg::Vec3f v)
{
	trans_ = v;
	this->setPosition(v);
}

osg::Vec3f myshape::getTranslation()
{
	return trans_;
}

void myshape::setNote(const std::string &s)
{
	note_ = s;
	std::cout << "Got note: " << note_ << std::endl;
}

const std::string& myshape::getNote() const
{
	return note_;
}

void myshape::setNum(int num)
{
	num_ = num;
	std::cout << "Got new num: " << num_ << std::endl;
}

int myshape::getNum() const
{
	return num_;
}
