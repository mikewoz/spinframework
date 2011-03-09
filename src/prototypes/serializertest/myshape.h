#ifndef __myshape_H
#define __myshape_H

#include <osg/PositionAttitudeTransform>

#define SHAPE_SCALE 1.0f

namespace mytest {


class MyShape : public osg::PositionAttitudeTransform
{

public:

	enum shapeType { NONE, SPHERE, BOX, CYLINDER, CAPSULE, CONE, PLANE };

	MyShape();
	MyShape(shapeType shp);
	~MyShape();

	void init();

	//void setTranslation(float x, float y, float z);
	void setTranslation(osg::Vec3f v);
	osg::Vec3f getTranslation();

	void setNote(const std::string&);
	const std::string& getNote() const;

	void setNum(int num);
	int getNum() const;


private:

	shapeType shape_;
	osg::Vec3f trans_;
	std::string note_;
	int num_;
};



} // end namespace
#endif
