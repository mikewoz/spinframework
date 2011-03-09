#include <iostream>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osgDB/ObjectWrapper>
#include <osgDB/InputStream>
#include <osgDB/OutputStream>

#include "myshape.h"
/*
REGISTER_OBJECT_WRAPPER( MyShape, // The unique wrapper name
                         new mytest::MyShape, // The proto
                         mytest::MyShape, // The class typename
                         "osg::Object osg::Node osg::Group osg::Transform osg::PositionAttitudeTransform mytest::MyShape" )  // The inheritance relations
{
	//ADD_VEC3_SERIALIZER( Translation, osg::Vec3() );
	ADD_STRING_SERIALIZER( Note, "v" );
	ADD_INT_SERIALIZER( Num, 0 );

}
*/

using namespace mytest;

MyShape::MyShape ()
{
	// let default be a sphere:
	shape_ = MyShape::SPHERE;
	note_ = "init";
	num_ = -1;
	init();
}

MyShape::MyShape (shapeType shape)
{
	shape_ = shape;
	init();
}

MyShape::~MyShape(){}


void MyShape::init()
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

//void MyShape::setTranslation(float x, float y, float z)
void MyShape::setTranslation(osg::Vec3f v)
{
	trans_ = v;
	this->setPosition(v);
}

osg::Vec3f MyShape::getTranslation()
{
	return trans_;
}

void MyShape::setNote(const std::string &s)
{
	note_ = s;
	std::cout << "Got note: " << note_ << std::endl;
}

const std::string& MyShape::getNote() const
{
	return note_;
}

void MyShape::setNum(int num)
{
	num_ = num;
	std::cout << "Got new num: " << num_ << std::endl;
}

int MyShape::getNum() const
{
	return num_;
}
