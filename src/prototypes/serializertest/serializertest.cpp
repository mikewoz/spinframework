#include <osgViewer/Viewer>
#include <osg/Group>

#include <iostream>
#include <cstdlib>
#include <cmath>

#include "myshape.h"

int main(int argc, char **argv)
{
	spinframework::myshape *sphere = new spinframework::myshape(spinframework::myshape::SPHERE);
	spinframework::myshape *box = new spinframework::myshape(spinframework::myshape::BOX);

	sphere->setTranslation(osg::Vec3(-1, 0, 0));
	box->setTranslation(osg::Vec3(1, 0, 0));

	sphere->setNote("test string");

	osg::Group *grp = new osg::Group();
	grp->addChild(sphere);
	grp->addChild(box);

	osgViewer::Viewer viewer;
	viewer.setSceneData( grp );
    return viewer.run();
}
